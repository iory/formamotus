import re  # Added for cleaning property names
from typing import ClassVar

import bpy
import numpy as np
from skrobot.data import pr2_urdfpath
from skrobot.model import RobotModel
from skrobot.utils.urdf import no_mesh_load_mode

from formamotus.utils.rendering_utils import enable_freestyle

_robot_model = None
_cylinder_objects = {}
_thin_cylinder_objects = []
_thin_cylinder_radius_scale = 0.03 * 0.3

def set_robot_model(model):
    global _robot_model
    _robot_model = model

def get_robot_model():
    return _robot_model


def update_cylinder_size(self, context):
    global _cylinder_objects
    scene = context.scene
    radius = scene.formamotus_cylinder_radius
    height = scene.formamotus_cylinder_height

    for _link, cylinder in _cylinder_objects.items():
        cylinder.scale = (radius / 0.03, radius / 0.03, height / 0.15)
    bpy.context.view_layer.update()

def update_connector_cylinder_size(self, context):
    global _thin_cylinder_objects
    global _thin_cylinder_radius_scale
    scene = context.scene
    radius = scene.formamotus_connector_cylinder_radius

    _thin_cylinder_radius_scale = radius / (0.03 * 0.3)
    for _org_parent_link, _link, thin_cylinder, _org_length in _thin_cylinder_objects:
        scale = thin_cylinder.scale
        thin_cylinder.scale = (_thin_cylinder_radius_scale,
                               _thin_cylinder_radius_scale, scale[2])
    bpy.context.view_layer.update()

def update_joint_position(self, context):
    global _cylinder_objects
    global _robot_model
    global _thin_cylinder_objects
    global _thin_cylinder_radius_scale
    if not _cylinder_objects or not _robot_model:
        return
    scene = context.scene

    for joint_name in _robot_model.joint_names:
        joint = _robot_model.__dict__.get(joint_name)
        if joint and joint.type != 'fixed':
            prop_name = f"formamotus_joint_angle_{joint_name.replace(' ', '_').replace('/', '_')}"
            if hasattr(scene, prop_name):
                joint_angle = getattr(scene, prop_name)
                joint.joint_angle(joint_angle)
    for link, cylinder in _cylinder_objects.items():
        # Update the position and rotation of the cylinder
        parent_link = link.copy_worldcoords()
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
        cylinder.location = parent_link.worldpos()
        cylinder.rotation_mode = 'QUATERNION'
        cylinder.rotation_quaternion = parent_link.quaternion

    # Update the position and rotation of the thin cylinders
    for org_parent_link, link, thin_cylinder, org_length in _thin_cylinder_objects:
        start_pos = org_parent_link.worldpos()
        end_pos = link.worldpos()
        direction = end_pos - start_pos
        length = np.linalg.norm(direction)

        if length > 1e-6:
            mid_pos = start_pos + direction * 0.5
            thin_cylinder.location = mid_pos
            thin_cylinder.scale = (_thin_cylinder_radius_scale, _thin_cylinder_radius_scale,
                                   length / org_length)

            # Compute the rotation axis and angle
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

    bpy.context.view_layer.update()


def register_custom_properties():
    """Register custom properties to the scene."""
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

    bpy.types.Scene.formamotus_cylinder_radius = bpy.props.FloatProperty(
        name="Cylinder Radius",
        description="Radius of the joint cylinders",
        default=0.03,
        min=0.000001, max=1.0,
        update=update_cylinder_size
    )

    bpy.types.Scene.formamotus_cylinder_height = bpy.props.FloatProperty(
        name="Cylinder Height",
        description="Height of the joint cylinders",
        default=0.15,
        min=0.000001, max=2.0,
        update=update_cylinder_size
    )

    bpy.types.Scene.formamotus_connector_cylinder_radius = bpy.props.FloatProperty(
        name="Connector Cylinder Radius",
        description="Radius of the joint cylinders for connector",
        default=0.03 * 0.3,
        min=0.00001, max=1.0,
        update=update_connector_cylinder_size
    )

def unregister_custom_properties():
    """Unregister custom properties from the scene."""
    del bpy.types.Scene.formamotus_urdf_filepath
    del bpy.types.Scene.formamotus_render_filepath
    del bpy.types.Scene.formamotus_revolute_color
    del bpy.types.Scene.formamotus_prismatic_color
    del bpy.types.Scene.formamotus_continuous_color
    del bpy.types.Scene.formamotus_default_color
    del bpy.types.Scene.formamotus_cylinder_radius
    del bpy.types.Scene.formamotus_cylinder_height

class RobotVisualizerOperator(bpy.types.Operator):
    bl_idname = "robot_viz.visualize_robot"
    bl_label = "Visualize Robot Model"
    bl_options: ClassVar[set[str]] = {'REGISTER', 'UNDO'}

    def clean_property_name(self, joint_name):
        """Clean joint name to create a valid property name."""
        # Replace spaces, slashes, and other special characters with underscores
        cleaned_name = re.sub(r'[^a-zA-Z0-9_]', '_', joint_name)
        return f"formamotus_joint_angle_{cleaned_name}"

    def add_joint_angle_properties(self, context):
        """Dynamically add joint angle properties based on the robot model."""
        global _robot_model
        if not _robot_model:
            return

        # Clear existing joint angle properties
        existing_props = [p for p in dir(bpy.types.Scene) if p.startswith("formamotus_joint_angle_")]
        for prop in existing_props:
            try:
                delattr(bpy.types.Scene, prop)
                print(f"Deleted property: {prop}")
            except Exception as e:
                print(f"Failed to delete property {prop}: {e}")

        # Add a property for each joint
        for joint_name in _robot_model.joint_names:
            joint = _robot_model.__dict__.get(joint_name)
            if joint and joint.type != 'fixed':
                # Get joint angle limits
                min_angle = joint.min_angle
                max_angle = joint.max_angle

                # Ensure min_angle and max_angle are finite and valid
                if not np.isfinite(min_angle) or min_angle is None:
                    min_angle = -np.pi
                if not np.isfinite(max_angle) or max_angle is None:
                    max_angle = np.pi

                # Ensure min_angle < max_angle
                if min_angle >= max_angle:
                    min_angle, max_angle = -np.pi, np.pi

                if joint.type == 'continuous':
                    min_angle, max_angle = -np.pi, np.pi  # Default range for continuous joints

                # Log min_angle and max_angle
                message = f"Joint: {joint_name}, min_angle: {min_angle}, max_angle: {max_angle}"
                self.report({'INFO'}, message)

                # Create a safe property name
                prop_name = self.clean_property_name(joint_name)
                self.report({'INFO'}, prop_name)

                if hasattr(bpy.types.Scene, prop_name):
                    print(f"{prop_name} already exists. Deleting...")
                    self.report({'INFO'}, f"{prop_name} already exists. Deleting...")
                    delattr(bpy.types.Scene, prop_name)

                # Add custom property for joint angle
                try:
                    setattr(
                        bpy.types.Scene,
                        prop_name,
                        bpy.props.FloatProperty(
                            name=f"{joint_name} Angle",
                            description=f"Angle for joint {joint_name}",
                            default=0.0,
                            min=min_angle,
                            max=max_angle,
                            update=update_joint_position,
                        )
                    )
                    print(f"Added property: {prop_name} (min: {min_angle}, max: {max_angle})")
                except Exception as e:
                    print(f"Failed to add property {prop_name}: {e}")
                    self.report({'INFO'}, f"Failed to add property {prop_name}: {e}")
                    raise

    def execute(self, context):
        global _cylinder_objects
        global _robot_model
        global _thin_cylinder_objects
        scene = context.scene
        render_filepath = scene.formamotus_render_filepath
        revolute_color = scene.formamotus_revolute_color
        prismatic_color = scene.formamotus_prismatic_color
        continuous_color = scene.formamotus_continuous_color
        default_color = scene.formamotus_default_color
        urdf_filepath = scene.formamotus_urdf_filepath

        _cylinder_objects = {}
        _thin_cylinder_objects = []
        _robot_model = None

        # Clear the scene
        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete()

        # Set up Freestyle and background
        enable_freestyle(line_thickness=3.0, line_color=(0, 0, 0))
        bpy.context.scene.world.use_nodes = True
        bg_node = bpy.context.scene.world.node_tree.nodes["Background"]
        bg_node.inputs[0].default_value = (1, 1, 1, 1)
        bg_node.inputs[1].default_value = 1.0

        # Load the robot model
        _robot_model = RobotModel()
        with no_mesh_load_mode():
            _robot_model.load_urdf_file(urdf_filepath)

        # Add joint angle properties
        self.add_joint_angle_properties(context)

        links = [(_robot_model.root_link, _robot_model.root_link, _robot_model.root_link.copy_worldcoords())]
        radius = 0.03
        height = 0.15

        while links:
            link, org_parent_link, parent_coords = links.pop()
            if link.joint is not None and link.joint.type != 'fixed':
                org_parent_coords = parent_coords.copy_worldcoords()
                parent_link = link.copy_worldcoords()
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

                mat = bpy.data.materials.new(name=f"Material_{link.name}")
                mat.use_nodes = True
                nodes = mat.node_tree.nodes
                principled = nodes.get("Principled BSDF")
                principled.inputs["Base Color"].default_value = random_color
                principled.inputs["Alpha"].default_value = 1.0
                mat.blend_method = 'BLEND'
                cylinder.data.materials.append(mat)

                # Map the cylinder to the link
                _cylinder_objects[link] = cylinder

                start_pos = org_parent_coords.worldpos()
                end_pos = parent_coords.worldpos()
                direction = end_pos - start_pos
                length = np.linalg.norm(direction)
                thin_radius = radius * 0.3

                if length > 1e-6:
                    mid_pos = start_pos + direction * 0.5
                    bpy.ops.mesh.primitive_cylinder_add(radius=thin_radius, depth=length, location=mid_pos)
                    thin_cylinder = bpy.context.object
                    thin_cylinder.name = f"Connector_{org_parent_link.name}_to_{link.name}"

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

                    thin_mat = bpy.data.materials.new(name=f"ConnectorMaterial_{org_parent_link.name}_to_{link.name}")
                    thin_mat.use_nodes = True
                    thin_nodes = thin_mat.node_tree.nodes
                    thin_principled = thin_nodes.get("Principled BSDF")
                    thin_principled.inputs["Base Color"].default_value = (0, 0, 0, 1)
                    thin_principled.inputs["Alpha"].default_value = 1.0
                    thin_mat.blend_method = 'BLEND'
                    thin_cylinder.data.materials.append(thin_mat)
                    _thin_cylinder_objects.append((
                        org_parent_link,
                        link,
                        thin_cylinder,
                        length,
                    ))
                org_parent_link = link
            for child_link in link.child_links:
                links.append((child_link, org_parent_link, parent_coords.copy_worldcoords()))

        # Add light and camera
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


def register():
    register_custom_properties()
    bpy.utils.register_class(RobotVisualizerOperator)


def unregister():
    unregister_custom_properties()
    bpy.utils.unregister_class(RobotVisualizerOperator)
