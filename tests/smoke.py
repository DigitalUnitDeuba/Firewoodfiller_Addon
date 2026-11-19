import runpy
from pathlib import Path

import bpy
from mathutils import Matrix, Vector

addon = runpy.run_path(str(Path(__file__).resolve().parents[1] / "__init__.py"))
addon["register"]()
scene = bpy.context.scene
settings = scene.fw_shelf_settings
settings.stack_mode = "PHYSICS"
assert settings.fill_height == 1.0
assert bpy.ops.fw_shelf.make_volume() == {'FINISHED'}
volume = settings.volume_object
volume.rotation_euler.z = 0.63
volume.location = (0.9, -0.3, 1.0)

source = bpy.data.collections.new("Brennholz")
scene.collection.children.link(source)
for i, sides in enumerate((5, 9)):
    bpy.ops.mesh.primitive_cylinder_add(vertices=sides, radius=0.1, depth=0.55)
    obj = bpy.context.object
    obj.name = f"Scheit {i}"
    obj.users_collection[0].objects.unlink(obj)
    source.objects.link(obj)
    obj.scale = (1, 0.82, 1)
settings.source_collection = source
settings.frames = 45
settings.substeps = 8

bpy.ops.mesh.primitive_cube_add(location=(8, 0, 0))
outside = bpy.context.object
bpy.ops.rigidbody.object_add(type='PASSIVE')
scene.rigidbody_world.substeps_per_frame = 9
scene.frame_set(17, subframe=0.25)
timeline = scene.frame_start, scene.frame_end, scene.frame_current_final
before = set(bpy.data.scenes), set(bpy.data.meshes), set(bpy.data.collections)

settings.fill_height = 0.50
assert bpy.ops.fw_shelf.generate() == {'FINISHED'}
first = addon["find_stack"](scene, volume)
assert first is not None
half_count = len(first.objects)
assert half_count > 8, half_count
assert all(obj.data in {src.data for src in source.objects} and obj.rigid_body is None for obj in first.objects)
frame, dims = addon["fill_geometry"](*addon["volume_geometry"](volume), settings.fill_height)
inv = frame.inverted()
bpy.context.view_layer.update()
for obj in first.objects:
    for vertex in obj.data.vertices:
        p = inv @ obj.matrix_world @ vertex.co
        assert all(abs(p[i]) <= dims[i] * 0.5 + 0.00101 for i in range(3)), p
assert scene.rigidbody_world.substeps_per_frame == 9
assert outside.rigid_body.type == 'PASSIVE'
assert bpy.context.view_layer.objects.active == outside and outside.select_get()
assert (scene.frame_start, scene.frame_end, scene.frame_current_final) == timeline
assert (set(bpy.data.scenes), set(bpy.data.meshes), set(bpy.data.collections) - {first}) == before

settings.fill_height = 0
try:
    bpy.ops.fw_shelf.generate()
except RuntimeError as exc:
    assert "Füllhöhe" in str(exc), exc
else:
    raise AssertionError("Zero height must fail")
assert addon["find_stack"](scene, volume) == first

volume.name = "Mein Regal"
settings.fill_height = 1
volume.scale.x *= 2
volume.scale.z *= 2
bpy.context.view_layer.update()
assert bpy.ops.fw_shelf.generate() == {'FINISHED'}
full = addon["find_stack"](scene, volume)
full_count = len(full.objects)
assert full_count > 450, full_count
bpy.context.view_layer.update()
frame, dims = addon["volume_geometry"](volume)
inverse = frame.inverted()
top = max((inverse @ obj.matrix_world @ vertex.co).z
          for obj in full.objects for vertex in obj.data.vertices)
reached_height = (top + dims.z / 2) / dims.z
assert reached_height >= 0.93, reached_height

logs = list(full.objects)[::full_count // 4]
relative = [volume.matrix_world.inverted() @ obj.matrix_world for obj in logs]
volume.location += Vector((2, -1, 3))
volume.rotation_euler.z -= 0.8
volume.scale.x *= 1.2
bpy.context.view_layer.update()
for obj, expected in zip(logs, relative):
    assert obj.parent == volume
    actual = volume.matrix_world.inverted() @ obj.matrix_world
    assert all(abs(actual[row][column] - expected[row][column]) < 0.00001
               for row in range(4) for column in range(4)), actual
settings.manual_count = 12
settings.length_mode = 'FIXED'
settings.log_length = 0.28
scene.unit_settings.system = 'METRIC'
variants = list(source.objects)
frame, dims = addon["volume_geometry"](volume)
inverse = frame.inverted()
setup = addon["prepare_variants"](variants, dims, settings)[0]
for outside_depth in (-1, 1):
    matrix = frame @ Matrix.Translation((0, outside_depth * 0.16, 0))
    offset, local = addon["fit_depth"](matrix, setup, frame, inverse, dims.y)
    assert offset is not None and offset.translation != matrix.translation
    depths = [(local @ vertex.co).y for vertex in setup[0].data.vertices]
    assert min(depths) >= -dims.y / 2 and max(depths) <= dims.y / 2
    assert abs(max(depths) - min(depths) - settings.log_length) < 0.00001

settings.log_length = 0.5
try:
    bpy.ops.fw_shelf.generate()
except RuntimeError as exc:
    assert "zu lang" in str(exc) and "Maximal" in str(exc), exc
else:
    raise AssertionError("A fixed, excessive length must fail")
assert addon["find_stack"](scene, volume) == full

for i, obj in enumerate(variants):
    obj.scale.z = 0.47 + i * 0.06
settings.log_length = 0.28
for mode in ('FIXED', 'SOURCE'):
    settings.length_mode = mode
    assert bpy.ops.fw_shelf.generate() == {'FINISHED'}
    stack = addon["find_stack"](scene, volume)
    bpy.context.view_layer.update()
    frame, dims = addon["volume_geometry"](volume)
    inverse = frame.inverted()
    for obj in stack.objects:
        local = inverse @ obj.matrix_world
        assert all(abs((local @ vertex.co).y) <= dims.y * 0.5 for vertex in obj.data.vertices)
        original = source.objects[obj["firewood_variant"]]
        local_length = max(v.co.z for v in original.data.vertices) - min(v.co.z for v in original.data.vertices)
        actual = (obj.matrix_world.to_3x3() @ Vector((0, 0, local_length))).length
        expected = settings.log_length if mode == 'FIXED' else local_length * original.scale.z
        assert abs(actual - expected) < 0.00001, (mode, actual, expected)

assert len(set(bpy.data.collections) - before[2]) == 1
assert bpy.ops.fw_shelf.clear() == {'FINISHED'}
assert addon["find_stack"](scene, volume) is None
assert (set(bpy.data.scenes), set(bpy.data.meshes), set(bpy.data.collections)) == before
addon["unregister"]()
print(f"PASS: 50% = {half_count} Scheite, 100% = {full_count} Scheite (Oberkante bei {reached_height:.1%})")
