import bpy
import numpy as np
from mathutils import Vector, Matrix
from skrobot.models import PR2
from skrobot.model import RobotModel, Link, Cylinder
from skrobot.coordinates import Coordinates

from formamotus.utils.rendering_utils import enable_freestyle

class RobotVisualizerOperator(bpy.types.Operator):
    bl_idname = "robot_viz.visualize_robot"
    bl_label = "Visualize Robot Model"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete()

        enable_freestyle(line_thickness=3.0, line_color=(0, 0, 0))
        bpy.context.scene.world.use_nodes = True
        bg_node = bpy.context.scene.world.node_tree.nodes["Background"]
        bg_node.inputs[0].default_value = (1, 1, 1, 1)
        bg_node.inputs[1].default_value = 1.0

        robot_model = PR2()
        robot_model.reset_pose()
        robot_model.init_pose()
        robot_model.gripper_distance(0.1)

        base = RobotModel()
        base.root_link = Link()
        base.assoc(base.root_link)

        valid_joint_names = [
            'fl_caster_rotation_joint', 'fl_caster_l_wheel_joint', 'fl_caster_r_wheel_joint',
            'fr_caster_rotation_joint', 'fr_caster_l_wheel_joint', 'fr_caster_r_wheel_joint',
            'bl_caster_rotation_joint', 'bl_caster_l_wheel_joint', 'bl_caster_r_wheel_joint',
            'br_caster_rotation_joint', 'br_caster_l_wheel_joint', 'br_caster_r_wheel_joint',
            'torso_lift_joint', 'head_pan_joint', 'head_tilt_joint',
            'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint',
            'r_forearm_roll_joint', 'r_elbow_flex_joint', 'r_wrist_flex_joint',
            'r_wrist_roll_joint', 'r_gripper_l_finger_tip_joint', 'r_gripper_r_finger_tip_joint',
            'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint',
            'l_forearm_roll_joint', 'l_elbow_flex_joint', 'l_wrist_flex_joint',
            'l_wrist_roll_joint', 'l_gripper_l_finger_tip_joint', 'l_gripper_r_finger_tip_joint',
        ]

        base_link_list = []
        links = [(robot_model.root_link, base.root_link, base.root_link.copy_worldcoords())]
        radius = 0.03
        height = 0.15

        while links:
            link, parent_link, parent_coords = links.pop()
            if link.joint is not None and link.joint.type != 'fixed' and link.joint.name in valid_joint_names:
                org_parent_coords = parent_coords.copy_worldcoords()
                parent_link = Cylinder(radius, height, vertex_colors=(0, 0, 0, 127))
                parent_link.newcoords(link.copy_worldcoords())
                parent_coords = parent_link.copy_worldcoords()

                axis = link.joint.axis
                if isinstance(axis, str):
                    axis_dict = {'x': [1, 0, 0], 'y': [0, 1, 0], 'z': [0, 0, 1]}
                    axis_vector = np.array(axis_dict.get(axis.lower(), [0, 0, 1]))
                else:
                    axis_vector = np.array(axis)
                default_vector = np.array([0, 0, 1])
                rotation_axis = np.cross(default_vector, axis_vector)

                if np.linalg.norm(rotation_axis) > 1e-6:
                    rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
                    angle = np.arccos(np.dot(default_vector, axis_vector))
                    parent_link.rotate(angle, rotation_axis)

                bpy.ops.mesh.primitive_cylinder_add(radius=radius, depth=height, location=parent_link.worldpos())
                cylinder = bpy.context.object
                cylinder.name = f"{link.joint.name}"
                cylinder.rotation_mode = 'QUATERNION'
                cylinder.rotation_quaternion = parent_link.copy_worldcoords().quaternion

                if link.joint.type == 'revolute':
                    random_color = (1, 0, 0, 1)
                elif link.joint.type == 'prismatic':
                    random_color = (0, 1, 0, 1)
                elif link.joint.type == 'continuous':
                    random_color = (0, 0, 1, 1)
                else:
                    random_color = (0, 0, 0, 1)

                mat = bpy.data.materials.new(name=f"CylinderMaterial_{len(base_link_list)}")
                mat.use_nodes = True
                nodes = mat.node_tree.nodes
                principled = nodes.get("Principled BSDF")
                principled.inputs["Base Color"].default_value = random_color
                principled.inputs["Alpha"].default_value = 1.0
                mat.blend_method = 'BLEND'
                cylinder.data.materials.append(mat)

                start_pos = org_parent_coords.worldpos()
                end_pos = parent_coords.worldpos()
                direction = end_pos - start_pos
                length = np.linalg.norm(direction)
                thin_radius = radius * 0.3

                if length > 1e-6:
                    mid_pos = start_pos + direction * 0.5
                    bpy.ops.mesh.primitive_cylinder_add(radius=thin_radius, depth=length, location=mid_pos)
                    thin_cylinder = bpy.context.object
                    thin_cylinder.name = f"ThinConnector_{len(base_link_list)}"

                    z_axis = np.array([0, 0, 1])
                    rot_axis = np.cross(z_axis, direction)
                    if np.linalg.norm(rot_axis) > 1e-6:
                        rot_axis = rot_axis / np.linalg.norm(rot_axis)
                        angle = np.arccos(np.dot(z_axis, direction) / length)
                        thin_cylinder.rotation_mode = 'AXIS_ANGLE'
                        thin_cylinder.rotation_axis_angle = [angle, rot_axis[0], rot_axis[1], rot_axis[2]]
                    else:
                        thin_cylinder.rotation_mode = 'QUATERNION'
                        thin_cylinder.rotation_quaternion = [1, 0, 0, 0]

                    thin_mat = bpy.data.materials.new(name=f"ThinCylinderMaterial_{len(base_link_list)}")
                    thin_mat.use_nodes = True
                    thin_nodes = thin_mat.node_tree.nodes
                    thin_principled = thin_nodes.get("Principled BSDF")
                    thin_principled.inputs["Base Color"].default_value = (0, 0, 0, 1)
                    thin_principled.inputs["Alpha"].default_value = 1.0
                    thin_mat.blend_method = 'BLEND'
                    thin_cylinder.data.materials.append(thin_mat)

                base_link_list.append(parent_link)
            for child_link in link.child_links:
                links.append((child_link, parent_link, parent_coords.copy_worldcoords()))

        bpy.ops.object.light_add(type='POINT', location=(3, -3, 10))
        light = bpy.context.object
        light.data.energy = 1000
        light.data.shadow_soft_size = 0
        light.data.use_shadow = False

        bpy.ops.object.camera_add(location=(3, -3, 3), rotation=(1.1, 0, 0.78))
        camera = bpy.context.object
        bpy.context.scene.camera = camera
        bpy.context.scene.render.image_settings.file_format = 'PNG'
        bpy.context.scene.render.filepath = "/tmp/render_output.png"
        bpy.ops.render.render(write_still=True)

        self.report({'INFO'}, "Robot visualization completed!")
        return {'FINISHED'}
