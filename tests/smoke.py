import runpy
from pathlib import Path

import bpy
from mathutils import Vector

addon = runpy.run_path(str(Path(__file__).resolve().parents[1] / "__init__.py"))
addon["register"]()
scene = bpy.context.scene
settings = scene.fw_shelf_settings
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
assert bpy.ops.fw_shelf.generate() == {'FINISHED'}
full = addon["find_stack"](scene, volume)
full_count = len(full.objects)
assert full_count > half_count, (full_count, half_count)
assert len(set(bpy.data.collections) - before[2]) == 1
assert bpy.ops.fw_shelf.clear() == {'FINISHED'}
assert addon["find_stack"](scene, volume) is None
assert (set(bpy.data.scenes), set(bpy.data.meshes), set(bpy.data.collections)) == before
addon["unregister"]()
print(f"PASS: 50% = {half_count} Scheite, 100% = {full_count} Scheite")
