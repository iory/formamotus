from collections import defaultdict
import os
from pathlib import Path
import re  # Added for cleaning property names
import tempfile
from typing import ClassVar

import bpy
from mathutils import Matrix
from mathutils import Vector
import numpy as np
from skrobot.data import fetch_urdfpath
from skrobot.model import RobotModel
from skrobot.utils.urdf import no_mesh_load_mode
from skrobot.utils.urdf import resolve_filepath

from formamotus.utils.dae import fix_up_axis_and_get_materials
from formamotus.utils.dae import zero_origin_dae
from formamotus.utils.rendering_utils import enable_freestyle

_robot_model = None
_cylinder_objects = {}
_thin_cylinder_objects = []
_mesh_objects = defaultdict(list)
_coordinates_offset = {}


def set_robot_model(model):
    global _robot_model
    _robot_model = model

def get_robot_model():
    return _robot_model


def update_cylinder_size(self, context):
    global _cylinder_objects
    scene = context.scene
    radius_m = scene.formamotus_cylinder_radius / 1000.0
    height_m = scene.formamotus_cylinder_height / 1000.0

    for _link, cylinder in _cylinder_objects.items():
        cylinder.scale = (radius_m / 0.03, radius_m / 0.03, height_m / 0.15)
    bpy.context.view_layer.update()

def update_connector_cylinder_size(self, context):
    global _thin_cylinder_objects
    scene = context.scene
    radius_m = scene.formamotus_connector_cylinder_radius / 1000.0
    radius_scale = radius_m / (0.03 * 0.3)
    for _org_parent_link, _link, thin_cylinder, _org_length in _thin_cylinder_objects:
        scale = thin_cylinder.scale
        thin_cylinder.scale = (radius_scale, radius_scale, scale[2])
    bpy.context.view_layer.update()

def update_joint_position(self, context):
    global _cylinder_objects
    global _robot_model
    global _thin_cylinder_objects
    global _mesh_objects
    global _coordinates_offset
    if not _cylinder_objects or not _robot_model:
        return
    scene = context.scene

    for joint_name in _robot_model.joint_names:
        joint = _robot_model.__dict__.get(joint_name)
        if joint and joint.type != 'fixed':
            prop_name = f"formamotus_joint_angle_{joint_name.replace(' ', '_').replace('/', '_')}"
            if hasattr(scene, prop_name):
                joint_angle_ui = getattr(scene, prop_name)
                if joint.type == 'prismatic':
                    joint.joint_angle(joint_angle_ui / 1000.0)
                elif joint.type in ['revolute', 'continuous']:
                    joint.joint_angle(np.deg2rad(joint_angle_ui))
                else:
                    joint.joint_angle(joint_angle_ui)

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
            thin_cylinder.scale = (thin_cylinder.scale[0], thin_cylinder.scale[1], length / org_length)
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

    for link, mesh_objs in _mesh_objects.items():
        link_coords = link.copy_worldcoords()
        if link in _coordinates_offset:
            link_coords = link_coords.copy_worldcoords().transform(_coordinates_offset[link])
        for mesh_obj in mesh_objs:
            mesh_obj.location = link_coords.worldpos()
            mesh_obj.rotation_mode = 'QUATERNION'
            mesh_obj.rotation_quaternion = link_coords.quaternion

    bpy.context.view_layer.update()


def register_custom_properties():
    """Register custom properties to the scene."""
    bpy.types.Scene.formamotus_urdf_filepath = bpy.props.StringProperty(
        name="URDF Filepath",
        description="Path to load URDF filepath",
        default=str(fetch_urdfpath()),
        subtype='FILE_PATH'
    )

    temp_dir = tempfile.gettempdir()
    temp_file_path = os.path.join(temp_dir, "render_output.png")
    bpy.types.Scene.formamotus_render_filepath = bpy.props.StringProperty(
        name="Render Filepath",
        description="Path to save the rendered image",
        default=temp_file_path,
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
        name="Cylinder Radius (mm)",
        description="Radius of the joint cylinders",
        default=30.0,
        min=0.001, max=1000.0,
        update=update_cylinder_size
    )

    bpy.types.Scene.formamotus_cylinder_height = bpy.props.FloatProperty(
        name="Cylinder Height (mm)",
        description="Height of the joint cylinders",
        default=150.0,
        min=0.001, max=2000.0,
        update=update_cylinder_size
    )

    bpy.types.Scene.formamotus_connector_cylinder_radius = bpy.props.FloatProperty(
        name="Connector Cylinder Radius (mm)",
        description="Radius of the joint cylinders for connector",
        default=9.0,
        min=0.001, max=1000.0,
        update=update_connector_cylinder_size
    )

    bpy.types.Scene.formamotus_use_mesh = bpy.props.BoolProperty(
        name="Use Mesh Visualization",
        description="Enable to use mesh files from URDF for visualization",
        default=True,
        update=update_visibility
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
    del bpy.types.Scene.formamotus_use_mesh


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
            mimic = _robot_model.urdf_robot_model.joint_map[joint_name].mimic
            if mimic is not None:
                continue
            if joint and joint.type != 'fixed':
                # Get joint angle limits
                min_angle = joint.min_angle
                max_angle = joint.max_angle

                # Ensure min_angle and max_angle are finite and valid
                if not np.isfinite(min_angle) or min_angle is None:
                    min_angle = 0.0 if joint.type == 'prismatic' else -np.pi
                if not np.isfinite(max_angle) or max_angle is None:
                    max_angle = 0.1 if joint.type == 'prismatic' else np.pi

                # Ensure min_angle < max_angle
                if min_angle >= max_angle:
                    if joint.type in ['revolute', 'continuous']:
                        min_angle, max_angle = -np.pi, np.pi
                    else:
                        min_angle, max_angle = 0.0, 0.1

                if joint.type == 'continuous':
                    min_angle, max_angle = -np.pi, np.pi  # Default range for continuous joints

                unit_name = ''
                if joint.type == 'prismatic':
                    # meter to mm
                    min_angle *= 1000.0
                    max_angle *= 1000.0
                    unit_name = ' (mm)'
                elif joint.type in ['revolute', 'continuous']:
                    # rad to degree
                    min_angle = np.degrees(min_angle)
                    max_angle = np.degrees(max_angle)
                    unit_name = ' (deg)'

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
                            name=f"{joint_name} Angle{unit_name}",
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

    def import_mesh(self, mesh_filepath, link_name, color=None, visual_origin=None):
        global _coordinates_offset
        ext = os.path.splitext(mesh_filepath)[1].lower()
        try:
            if ext == '.stl':
                if "stl_import" in dir(bpy.ops.wm):
                    bpy.ops.wm.stl_import(
                        filepath=mesh_filepath,
                        up_axis='Z', forward_axis='Y', global_scale=1.0)
                elif "stl" in dir(bpy.ops.import_mesh):
                    bpy.ops.import_mesh.stl(
                        filepath=mesh_filepath, global_scale=1.0)
                else:
                    self.report({'WARNING'}, "STL import is not supported")
                    return None
            elif ext == '.dae':
                if visual_origin is not None:
                    file_path = zero_origin_dae(mesh_filepath, visual_origin)
                    bpy.ops.wm.collada_import(filepath=file_path)
                else:
                    (file_path, _) = fix_up_axis_and_get_materials(mesh_filepath)
                    bpy.ops.wm.collada_import(filepath=file_path)
                    Path(file_path).unlink()
            elif ext == '.obj':
                if "obj_import" in dir(bpy.ops.wm):
                    bpy.ops.wm.obj_import(
                        filepath=mesh_filepath,
                        up_axis='Z', forward_axis='Y', global_scale=1.0)
                elif "obj" in dir(bpy.ops.import_mesh):
                    bpy.ops.import_scene.obj(
                        filepath=mesh_filepath, axis_forward="Y", axis_up="Z")
                else:
                    self.report({'WARNING'}, "OBJ import is not supported")
                    return None
            else:
                self.report({'WARNING'}, f"Unsupported mesh format: {ext} for {link_name}")
                return None

            if not bpy.context.object.data.uv_layers:
                bpy.ops.mesh.uv_texture_add()

            imported_objects = list(bpy.context.selected_objects)
            if not imported_objects:
                self.report({'WARNING'}, f"Failed to import mesh: {mesh_filepath}")
                return None

            mesh_obj_list = []
            for i, mesh_obj in enumerate(bpy.context.selected_objects):
                mesh_obj.name = f"Mesh_{link_name}_{i}"

                # Apply color only if specified
                if color is not None:
                    # Create a new material with the specified color
                    material = bpy.data.materials.new(name=f"Material_{link_name}_{i}")
                    material.use_nodes = True
                    principled_bsdf = material.node_tree.nodes.get("Principled BSDF")
                    if principled_bsdf:
                        principled_bsdf.inputs["Base Color"].default_value = color

                    # Assign the material to the mesh
                    if mesh_obj.data.materials:
                        mesh_obj.data.materials[0] = material
                    else:
                        mesh_obj.data.materials.append(material)
                mesh_obj_list.append(mesh_obj)
            return mesh_obj_list
        except Exception as e:
            self.report({'WARNING'}, f"Error importing mesh {mesh_filepath}: {e}")
            return None

    def resolve_mesh_filepath(self, urdf_filepath, mesh_filename):
        urdf_dir = os.path.dirname(urdf_filepath)
        mesh_filepath = resolve_filepath(urdf_dir, mesh_filename)
        return mesh_filepath

    def execute(self, context):
        global _cylinder_objects
        global _robot_model
        global _thin_cylinder_objects
        global _mesh_objects
        global _coordinates_offset
        scene = context.scene
        revolute_color = scene.formamotus_revolute_color
        prismatic_color = scene.formamotus_prismatic_color
        continuous_color = scene.formamotus_continuous_color
        default_color = scene.formamotus_default_color
        urdf_filepath = scene.formamotus_urdf_filepath
        use_mesh = scene.formamotus_use_mesh

        _cylinder_objects = {}
        _thin_cylinder_objects = []
        _mesh_objects = defaultdict(list)
        _coordinates_offset = {}
        _robot_model = None

        # Clear the scene
        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete()

        bpy.context.scene.world.use_nodes = True
        bg_node = bpy.context.scene.world.node_tree.nodes["Background"]
        bg_node.inputs[0].default_value = (1, 1, 1, 1)
        bg_node.inputs[1].default_value = 1.0

        # Load the robot model
        _robot_model = RobotModel()
        with no_mesh_load_mode():
            _robot_model.load_urdf_file(urdf_filepath)
        _robot_model.init_pose()

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
                cylinder.name = f"CylinderLink_{link.joint.name}"
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
                nodes.clear()  # Clear default nodes
                emission = nodes.new("ShaderNodeEmission")
                output = nodes.new("ShaderNodeOutputMaterial")
                emission.inputs["Color"].default_value = random_color
                emission.inputs["Strength"].default_value = 1.0
                mat.node_tree.links.new(emission.outputs["Emission"], output.inputs["Surface"])
                cylinder.data.materials.append(mat)

                if use_mesh is True:
                    cylinder.hide_viewport = True
                    cylinder.hide_render = True
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

                    if use_mesh is True:
                        thin_cylinder.hide_viewport = True
                        thin_cylinder.hide_render = True
                    # Use Emission shader for connectors
                    thin_mat = bpy.data.materials.new(name=f"ConnectorMaterial_{org_parent_link.name}_to_{link.name}")
                    thin_mat.use_nodes = True
                    thin_nodes = thin_mat.node_tree.nodes
                    thin_nodes.clear()
                    thin_emission = thin_nodes.new("ShaderNodeEmission")
                    thin_output = thin_nodes.new("ShaderNodeOutputMaterial")
                    thin_emission.inputs["Color"].default_value = (0, 0, 0, 1)  # Black for connectors
                    thin_emission.inputs["Strength"].default_value = 1.0
                    thin_mat.node_tree.links.new(thin_emission.outputs["Emission"], thin_output.inputs["Surface"])
                    thin_cylinder.data.materials.append(thin_mat)
                    _thin_cylinder_objects.append((
                        org_parent_link,
                        link,
                        thin_cylinder,
                        length,
                    ))
                org_parent_link = link

            # Load mesh if enabled
            urdf_link = _robot_model.urdf_robot_model.link_map[link.name]
            if hasattr(urdf_link, 'visuals') and urdf_link.visuals:
                for i_visual, visual in enumerate(urdf_link.visuals):
                    if hasattr(visual, 'origin'):
                        visual_origin = visual.origin
                    else:
                        visual_origin = None
                    if hasattr(visual.geometry, 'mesh') and visual.geometry.mesh and visual.geometry.mesh.filename:
                        mesh_filepath = self.resolve_mesh_filepath(urdf_filepath, visual.geometry.mesh.filename)
                        self.report({'INFO'}, f"{mesh_filepath}")
                        if os.path.exists(mesh_filepath):
                            color = None
                            mesh_obj_list = self.import_mesh(mesh_filepath, link.name, color=color,
                                                             visual_origin=visual_origin)
                            for i_mesh, mesh_obj in enumerate(mesh_obj_list):
                                # Set position and rotation
                                link_coords = link.copy_worldcoords()
                                mesh_obj.location = link_coords.worldpos()
                                mesh_obj.rotation_mode = 'QUATERNION'
                                mesh_obj.rotation_quaternion = link_coords.quaternion

                                # Assign material (simple gray emission for now)
                                name = f"MeshMaterial_{link.name}_{i_visual!s}_{i_mesh!s}"
                                mesh_mat = bpy.data.materials.new(name=name)
                                mesh_mat.use_nodes = True
                                mesh_nodes = mesh_mat.node_tree.nodes
                                mesh_nodes.clear()
                                mesh_emission = mesh_nodes.new("ShaderNodeEmission")
                                mesh_output = mesh_nodes.new("ShaderNodeOutputMaterial")
                                mesh_emission.inputs["Color"].default_value = (0.5, 0.5, 0.5, 1.0)  # Gray
                                mesh_emission.inputs["Strength"].default_value = 1.0
                                mesh_mat.node_tree.links.new(mesh_emission.outputs["Emission"], mesh_output.inputs["Surface"])
                                if mesh_obj.data:
                                    mesh_obj.data.materials.append(mesh_mat)
                                if use_mesh is False:
                                    mesh_obj.hide_viewport = True
                                    mesh_obj.hide_render = True
                                _mesh_objects[link].append(mesh_obj)
                        else:
                            self.report({'WARNING'}, f"Mesh file not found: {mesh_filepath}")
            for child_link in link.child_links:
                links.append((child_link, org_parent_link, parent_coords.copy_worldcoords()))

        self.report({'INFO'}, "Robot visualization completed!")
        return {'FINISHED'}


class RobotRenderOperator(bpy.types.Operator):
    bl_idname = "robot_viz.render_robot"
    bl_label = "Render Robot Image"
    bl_options: ClassVar[set[str]] = {'REGISTER', 'UNDO'}

    def setup_camera_and_light(self, context, center, size):
        """Set up camera and light to view the robot from right-front, 45 degrees above."""

        # Remove existing camera if it exists
        if context.scene.camera:
            bpy.data.objects.remove(context.scene.camera, do_unlink=True)

        # Camera settings
        distance = size * 2.0  # Distance to fit the robot comfortably
        angle_rad = np.deg2rad(45)
        camera_x = center[0] + distance * np.cos(angle_rad)  # Right-front in XY plane
        camera_y = center[1] + distance * np.cos(angle_rad)
        camera_z = center[2] + distance * np.sin(angle_rad)  # 45 degrees up

        # Create camera manually instead of using bpy.ops
        camera_data = bpy.data.cameras.new(name="RobotCamera")
        camera = bpy.data.objects.new("RobotCamera", camera_data)
        camera.location = (camera_x, camera_y, camera_z)
        camera.data.lens = 35
        camera.data.clip_end = distance * 2
        context.scene.collection.objects.link(camera)  # Link to scene
        context.scene.camera = camera  # Set as active camera

        # Camera position and target center
        camera_x, camera_y, camera_z = center[0] + size * 2, center[1] + size * 2, center[2] + size * 2
        camera_pos = Vector((camera_x, camera_y, camera_z))
        center = Vector(center)

        # Direction from camera to center
        direction = center - camera_pos
        if direction.length > 1e-6:
            direction.normalize()
            up = Vector((0, 0, 1))

            z_axis = -direction
            x_axis = up.cross(z_axis).normalized()
            y_axis = z_axis.cross(x_axis).normalized()

            mat_rot = Matrix((
                (x_axis.x, y_axis.x, z_axis.x, 0),
                (x_axis.y, y_axis.y, z_axis.y, 0),
                (x_axis.z, y_axis.z, z_axis.z, 0),
                (0,        0,        0,        1)
            ))

            camera = context.scene.camera
            camera.matrix_world = mat_rot
            camera.location = camera_pos

        # Light settings
        bpy.ops.object.light_add(type='POINT', location=(camera_x, camera_y, camera_z + 2))
        light = bpy.context.object
        light.data.energy = 1000
        light.data.shadow_soft_size = 0
        light.data.use_shadow = False

        # Set the camera as the active camera
        context.scene.camera = camera

    def render_scene(self, context, render_filepath):
        """Render the scene and save the output to the specified filepath."""
        global _cylinder_objects
        global _mesh_objects
        # Set up Freestyle and background
        use_mesh = context.scene.formamotus_use_mesh
        if use_mesh is False:
            enable_freestyle(line_thickness=3.0, line_color=(0, 0, 0))
        else:
            bpy.context.scene.render.use_freestyle = False
        # Calculate the bounding box of the robot
        min_coords = np.array([float('inf')] * 3)
        max_coords = np.array([-float('inf')] * 3)
        for link in _cylinder_objects.keys():
            pos = link.worldpos()
            min_coords = np.minimum(min_coords, pos)
            max_coords = np.maximum(max_coords, pos)
        center = (min_coords + max_coords) / 2
        size = np.max(max_coords - min_coords)
        self.setup_camera_and_light(context, center, size)
        bpy.context.scene.render.film_transparent = True
        bpy.context.scene.render.image_settings.file_format = 'PNG'
        bpy.context.scene.render.filepath = render_filepath
        bpy.ops.render.render(write_still=True)
        self.report({'INFO'}, "Rendering completed!")

    def execute(self, context):
        scene = context.scene
        render_filepath = scene.formamotus_render_filepath
        # Reuse the render_scene method from RobotVisualizerOperator
        self.render_scene(context, render_filepath)
        return {'FINISHED'}

def update_visibility(self, context):
    global _cylinder_objects
    global _mesh_objects
    global _thin_cylinder_objects
    scene = context.scene
    use_mesh = scene.formamotus_use_mesh
    if use_mesh:
        for cylinder in _cylinder_objects.values():
            cylinder.hide_viewport = True
            cylinder.hide_render = True
        for _, _, cylinder, _ in _thin_cylinder_objects:
            cylinder.hide_viewport = True
            cylinder.hide_render = True
        for _, mesh_objs in _mesh_objects.items():
            for mesh_obj in mesh_objs:
                mesh_obj.hide_viewport = False
                mesh_obj.hide_render = False
    else:
        for cylinder in _cylinder_objects.values():
            cylinder.hide_viewport = False
            cylinder.hide_render = False
        for _, _, cylinder, _ in _thin_cylinder_objects:
            cylinder.hide_viewport = False
            cylinder.hide_render = False
        for _, mesh_objs in _mesh_objects.items():
            for mesh_obj in mesh_objs:
                mesh_obj.hide_viewport = True
                mesh_obj.hide_render = True
    bpy.context.view_layer.update()

def register():
    register_custom_properties()
    bpy.utils.register_class(RobotVisualizerOperator)
    bpy.utils.register_class(RobotRenderOperator)

def unregister():
    unregister_custom_properties()
    bpy.utils.unregister_class(RobotVisualizerOperator)
    bpy.utils.unregister_class(RobotRenderOperator)
