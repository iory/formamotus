
import bpy
from skrobot.data import pr2_urdfpath

# Ensure the addon is enabled
addon_name = "FormaMotus"
if addon_name not in bpy.context.preferences.addons:
    bpy.ops.preferences.addon_enable(module="formamotus")

# Clear the scene
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete()

# Set up scene properties
scene = bpy.context.scene
scene.formamotus_urdf_filepath = str(bpy.path.abspath(str(pr2_urdfpath())))
scene.formamotus_render_filepath = "/tmp/render_output.png"

# Execute "Visualize Robot"
bpy.ops.robot_viz.visualize_robot()

# Execute "Render Robot"
bpy.ops.robot_viz.render_robot()

print("Script execution completed.")
