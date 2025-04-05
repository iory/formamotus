import bpy


def enable_freestyle(line_thickness=2.0, line_color=(0, 0, 0)):
    bpy.context.scene.render.use_freestyle = True

    if not bpy.context.scene.view_layers[0].freestyle_settings.linesets:
        lineset = bpy.context.scene.view_layers[0].freestyle_settings.linesets.new(name="Outline")
    else:
        lineset = bpy.context.scene.view_layers[0].freestyle_settings.linesets[0]

    linestyle = bpy.data.linestyles.get("LineStyle") or bpy.data.linestyles.new("LineStyle")
    linestyle.thickness = line_thickness
    linestyle.color = line_color

    lineset.linestyle = linestyle
    lineset.select_silhouette = True
    lineset.select_border = True
    lineset.select_crease = True
    lineset.select_external_contour = True
