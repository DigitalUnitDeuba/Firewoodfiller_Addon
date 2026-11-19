import runpy
from pathlib import Path
from statistics import mean, pstdev

import bpy

addon = runpy.run_path(str(Path(__file__).resolve().parents[1] / "__init__.py"))
addon["register"]()
scene = bpy.context.scene
settings = scene.fw_shelf_settings
settings.stack_mode = 'PHYSICS'
bpy.ops.fw_shelf.make_volume()
volume = settings.volume_object
volume.scale = (0.72, 0.40, 0.65)
volume.rotation_euler.z = 0.58

source = bpy.data.collections.new("Testscheite")
scene.collection.children.link(source)
for sides in (5, 8):
    bpy.ops.mesh.primitive_cylinder_add(vertices=sides, radius=0.10, depth=0.32)
    obj = bpy.context.object
    obj.users_collection[0].objects.unlink(obj)
    source.objects.link(obj)
    obj.scale.x, obj.scale.y = 1.15, 0.82
settings.source_collection = source
settings.manual_count = 95
settings.frames = 100
settings.substeps = 10


def measure(compaction, alignment):
    settings.compaction = compaction
    settings.front_alignment = alignment
    result, discarded = addon["main"](bpy.context, settings)
    assert len(result.objects) == settings.manual_count and discarded == 0
    bpy.context.view_layer.update()
    frame, dims = addon["volume_geometry"](volume)
    inverse = frame.inverted()
    fronts, heights = [], []
    for obj in result.objects:
        local = inverse @ obj.matrix_world
        depths = [(local @ vertex.co).y + dims.y * 0.5 for vertex in obj.data.vertices]
        assert 0 <= min(depths) <= max(depths) <= dims.y
        fronts.append(min(depths))
        heights.append(local.translation.z + dims.z * 0.5)
    return mean(fronts), pstdev(fronts), mean(heights)


compaction, alignment = settings.compaction, settings.front_alignment
natural = measure(0.0, 0.0)
uncompacted = measure(0.0, alignment)
aligned = measure(compaction, alignment)
assert aligned[0] < natural[0] * 0.25, (natural, aligned)
assert aligned[1] < natural[1] * 0.5, (natural, aligned)
assert aligned[2] < uncompacted[2], (uncompacted, aligned)
print(f"PHYSICS STYLE PASS: front spacing {natural[0]:.4f} -> {aligned[0]:.4f}, "
      f"mean log height {uncompacted[2]:.4f} -> {aligned[2]:.4f}")
