import bpy
from mathutils import Vector

def cylinders_overlap(obj1, obj2):
    bbox1 = [obj1.matrix_world @ Vector(corner) for corner in obj1.bound_box]
    bbox2 = [obj2.matrix_world @ Vector(corner) for corner in obj2.bound_box]

    min1 = Vector(map(min, zip(*bbox1)))
    max1 = Vector(map(max, zip(*bbox1)))
    min2 = Vector(map(min, zip(*bbox2)))
    max2 = Vector(map(max, zip(*bbox2)))

    for i in range(3):
        if max1[i] < min2[i] or min1[i] > max2[i]:
            return False
    return True

def merge_overlapping_cylinders():
    def is_valid(obj):
        return obj and obj.name in bpy.data.objects

    while True:
        cylinders = [
            obj for obj in bpy.data.objects
            if obj.type == 'MESH' and 'Cylinder' in obj.name and not obj.name.startswith("ThinConnector")
        ]
        merged = False

        for i in range(len(cylinders)):
            if not is_valid(cylinders[i]):
                continue
            for j in range(i + 1, len(cylinders)):
                if not is_valid(cylinders[j]):
                    continue

                obj1 = cylinders[i]
                obj2 = cylinders[j]

                if cylinders_overlap(obj1, obj2):
                    print(f"Resizing and merging {obj1.name} and {obj2.name}")
                    obj2.scale = (obj2.scale[0] * 0.8, obj2.scale[1] * 0.8, obj2.scale[2] * 0.8)
                    bpy.context.view_layer.update()

                    bool_mod = obj1.modifiers.new(name="BoolUnion", type='BOOLEAN')
                    bool_mod.operation = 'UNION'
                    bool_mod.object = obj2

                    bpy.context.view_layer.objects.active = obj1
                    bpy.ops.object.select_all(action='DESELECT')
                    obj1.select_set(True)

                    try:
                        bpy.ops.object.modifier_apply(modifier=bool_mod.name)
                    except RuntimeError:
                        print(f"Failed to apply boolean modifier for {obj1.name}")
                        continue

                    try:
                        bpy.data.objects.remove(obj2, do_unlink=True)
                    except:
                        pass

                    merged = True
                    break
            if merged:
                break

        if not merged:
            break

    bpy.context.view_layer.update()
    print("Cylinder merging completed.")
