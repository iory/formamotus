import bpy
import numpy as np
from skrobot.model import Cylinder
from skrobot.model import Link
from skrobot.model import RobotModel
from skrobot.models import PR2
from skrobot.data import pr2_urdfpath
from skrobot.utils.urdf import no_mesh_load_mode

from formamotus.utils.rendering_utils import enable_freestyle


def register_custom_properties():
    bpy.types.Scene.formamotus_urdf_filepath = bpy.props.StringProperty(
        name="URDF Filepath",
        description="Path to load URDF filepath",
        default=str(pr2_urdfpath()),
        subtype='FILE_PATH'
    )

    bpy.types.Scene.formamotus_render_filepath = bpy.props.StringProperty(
        name="Render Filepath",
        description="Path to save the rendered image",
        default="/tmp/render_output.png",
        subtype='FILE_PATH'
    )

    bpy.types.Scene.formamotus_revolute_color = bpy.props.FloatVectorProperty(
        name="Revolute Joint Color",
        description="Color for revolute joints",
        default=(1.0, 0.0, 0.0, 1.0),  # red
        min=0.0, max=1.0,
        subtype='COLOR',
        size=4
    )

    bpy.types.Scene.formamotus_prismatic_color = bpy.props.FloatVectorProperty(
        name="Prismatic Joint Color",
        description="Color for prismatic joints",
        default=(0.0, 1.0, 0.0, 1.0),  # green
        min=0.0, max=1.0,
        subtype='COLOR',
        size=4
    )

    bpy.types.Scene.formamotus_continuous_color = bpy.props.FloatVectorProperty(
        name="Continuous Joint Color",
        description="Color for continuous joints",
        default=(0.0, 0.0, 1.0, 1.0),  # blue
        min=0.0, max=1.0,
        subtype='COLOR',
        size=4
    )

    bpy.types.Scene.formamotus_default_color = bpy.props.FloatVectorProperty(
        name="Default Joint Color",
        description="Color for other joint types",
        default=(0.0, 0.0, 0.0, 1.0),  # black
        min=0.0, max=1.0,
        subtype='COLOR',
        size=4
    )


def unregister_custom_properties():
    del bpy.types.Scene.formamotus_urdf_filepath
    del bpy.types.Scene.formamotus_render_filepath
    del bpy.types.Scene.formamotus_revolute_color
    del bpy.types.Scene.formamotus_prismatic_color
    del bpy.types.Scene.formamotus_continuous_color
    del bpy.types.Scene.formamotus_default_color


class RobotVisualizerOperator(bpy.types.Operator):
    bl_idname = "robot_viz.visualize_robot"
    bl_label = "Visualize Robot Model"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        scene = context.scene
        render_filepath = scene.formamotus_render_filepath
        revolute_color = scene.formamotus_revolute_color
        prismatic_color = scene.formamotus_prismatic_color
        continuous_color = scene.formamotus_continuous_color
        default_color = scene.formamotus_default_color
        urdf_filepath = scene.formamotus_urdf_filepath

        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete()

        enable_freestyle(line_thickness=3.0, line_color=(0, 0, 0))
        bpy.context.scene.world.use_nodes = True
        bg_node = bpy.context.scene.world.node_tree.nodes["Background"]
        bg_node.inputs[0].default_value = (1, 1, 1, 1)
        bg_node.inputs[1].default_value = 1.0

        robot_model = RobotModel()
        with no_mesh_load_mode():
            robot_model.load_urdf_file(urdf_filepath)

        base = RobotModel()
        base.root_link = Link()
        base.assoc(base.root_link)

        base_link_list = []
        links = [(robot_model.root_link, base.root_link, base.root_link.copy_worldcoords())]
        radius = 0.03
        height = 0.15

        while links:
            link, parent_link, parent_coords = links.pop()
            if link.joint is not None and link.joint.type != 'fixed':
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
                    random_color = revolute_color
                elif link.joint.type == 'prismatic':
                    random_color = prismatic_color
                elif link.joint.type == 'continuous':
                    random_color = continuous_color
                else:
                    random_color = default_color

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
        bpy.context.scene.render.filepath = render_filepath
        bpy.ops.render.render(write_still=True)

        self.report({'INFO'}, "Robot visualization completed!")
        return {'FINISHED'}
