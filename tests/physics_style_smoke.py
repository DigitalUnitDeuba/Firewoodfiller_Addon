import runpy
from pathlib import Path
from statistics import mean, pstdev
from math import cos, radians

import bpy
from mathutils import Matrix, Vector

addon = runpy.run_path(str(Path(__file__).resolve().parents[1] / "__init__.py"))
addon["register"]()
scene = bpy.context.scene
settings = scene.fw_shelf_settings
settings.stack_mode = 'PHYSICS'
settings.section_scale_x = settings.section_scale_z = 1.0
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
settings.longitudinal_guide = False
natural = measure(0.0, 0.0)
uncompacted = measure(0.0, alignment)
aligned = measure(compaction, alignment)
assert aligned[0] < natural[0] * 0.25, (natural, aligned)
assert aligned[1] < natural[1] * 0.5, (natural, aligned)
assert aligned[2] < uncompacted[2], (uncompacted, aligned)
print(f"PHYSICS STYLE PASS: front spacing {natural[0]:.4f} -> {aligned[0]:.4f}, "
      f"mean log height {uncompacted[2]:.4f} -> {aligned[2]:.4f}")

volume.scale.y = 0.68
settings.length_mode = 'FIXED'
settings.log_length = 0.25
loose = measure(compaction, alignment)
settings.longitudinal_guide = True
guided = measure(compaction, alignment)
stack = addon["find_stack"](scene, volume)
frame, _ = addon["volume_geometry"](volume)
inverse = frame.inverted()
axes = [(inverse @ obj.matrix_world).to_3x3() @ Vector((0, 0, 0.32)) for obj in stack.objects]
assert all(abs(axis.normalized().y) > cos(radians(8)) for axis in axes)
assert all(abs(axis.length - settings.log_length) < 0.00001 for axis in axes)
assert guided[1] < loose[1] * 0.5, (loose, guided)
print(f"GUIDANCE PASS: minimum depth-axis alignment {min(abs(axis.normalized().y) for axis in axes):.4f}")

bpy.context.view_layer.update()
frame, dims = addon["volume_geometry"](volume)
variants = addon["source_variants"](source)
original_setups = addon["prepare_variants"](variants, dims, settings)
settings.section_scale_x, settings.section_scale_z = 0.65, 0.85
setups = addon["prepare_variants"](variants, dims, settings)
for (obj, original), (_, scaled) in zip(original_setups, setups):
    for vertex in obj.data.vertices:
        before, after = original @ vertex.co, scaled @ vertex.co
        assert abs(after.x - before.x * 0.65) < 0.00001
        assert abs(after.y - before.y) < 0.00001
        assert abs(after.z - before.z * 0.85) < 0.00001

floor = -dims.z * 0.5
placements = [(frame @ Matrix.Translation((x, 0, floor + z)), setups[0])
              for x in (-0.11, 0, 0.10) for z in (0.025, 0.0925)]
fillers = addon["fill_physics_gaps"](frame, dims, settings, setups, placements)
assert len(fillers) >= 3, len(fillers)
inverse = frame.inverted()
outline_for = addon["ordered_outline"]
edge_for = addon["profile_edges"]
contact = addon["contact_height"]
outlines = []
for index, (matrix, (obj, transform)) in enumerate(placements + fillers):
    local = inverse @ matrix @ transform
    points = [local @ vertex.co for vertex in obj.data.vertices]
    assert abs(max(p.y for p in points) - min(p.y for p in points) - 0.25) < 0.00001
    if index >= len(placements):
        assert all(abs(p.x) <= dims.x / 2 and abs(p.y) <= dims.y / 2
                   and floor <= p.z < floor + 0.12 for p in points)
    lower, upper = edge_for(outline_for([Vector((point.x, point.z)) for point in points]))
    if index >= len(placements):
        for old_lower, old_upper in outlines:
            above = contact(lower, old_upper, 0)
            below = contact(old_lower, upper, 0)
            assert above is None or below is None or min(above, below) <= 0.00001
    outlines.append((lower, upper))
print(f"GAP FILL PASS: {len(fillers)} supported pieces, each still 0.25 units long")
