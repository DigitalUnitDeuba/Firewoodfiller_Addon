import runpy
from pathlib import Path
from types import SimpleNamespace

import bpy
from mathutils import Vector

addon = runpy.run_path(str(Path(__file__).resolve().parents[1] / "__init__.py"))
addon["register"]()
scene = bpy.context.scene
settings = scene.fw_shelf_settings
assert settings.stack_mode == 'PRODUCT'
assert bpy.ops.fw_shelf.make_volume() == {'FINISHED'}
volume = settings.volume_object
volume.scale.x *= 2
volume.scale.z *= 2
volume.rotation_euler.z = 0.65
source = bpy.data.collections.new("Produktbild-Quellen")
scene.collection.children.link(source)
for sides in (5, 11):
    bpy.ops.mesh.primitive_cylinder_add(vertices=sides, radius=0.1, depth=0.5, location=(5, 2, 0))
    obj = bpy.context.object
    obj.users_collection[0].objects.unlink(obj)
    source.objects.link(obj)
    if sides == 11:
        obj.scale.x = 0.64
        obj.modifiers.new("Vorschau verwendet das Originalmesh", 'SUBSURF')
settings.source_collection = source
settings.length_mode = 'FIXED'
settings.log_length = 0.28
bpy.context.view_layer.update()
frame, dims = addon["volume_geometry"](volume)
setups = addon["prepare_variants"](list(source.objects), dims, settings)
preview = SimpleNamespace(
    frame=frame, dims=dims, height=1.0, count=None, settings=settings, setups=setups,
    scene=scene, volume=volume, preview_sources=[],
)
editor = addon["FWSHELF_OT_adjust_height"]
preview.populate_preview_points = lambda height: editor.populate_preview_points(preview, height)
editor.build_preview(preview)
tree = preview.preview_tree
output = next(node for node in tree.nodes if node.bl_idname == 'NodeGroupOutput')
socket = output.inputs["Geometry"]
realize = tree.nodes.new('GeometryNodeRealizeInstances')
tree.links.new(socket.links[0].from_socket, realize.inputs["Geometry"])
tree.links.new(realize.outputs["Geometry"], socket)

counts = []
for height in (1.0, 0.5, 0.0, 1.35, 1.9, 1.35):
    preview.height = height
    editor.update_preview(preview)
    depsgraph = bpy.context.evaluated_depsgraph_get()
    actual = len(preview.preview.evaluated_get(depsgraph).data.vertices)
    variants = preview.preview.data.attributes["fw_variant"].data
    expected = sum(len(setups[variants[i].value][0].data.vertices) for i in range(preview.count))
    assert actual == expected, (bpy.app.version_string, height, actual, expected)
    counts.append(preview.count)
assert counts[0] > 450
assert 0 == counts[2] < counts[1] < counts[0] < counts[3] < counts[4]
assert counts[3] == counts[5]

settings.fill_height = 1.35
assert settings.fill_height > 1.0
result, discarded = addon["main"](bpy.context, settings)
assert len(result.objects) == counts[3] and not discarded
assert scene.rigidbody_world is None
bpy.context.view_layer.update()
inverse = frame.inverted()
floor, ceiling = -dims.z * 0.5, dims.z * (settings.fill_height - 0.5)
bottom, top = float('inf'), -float('inf')
for obj in result.objects:
    local = inverse @ obj.matrix_world
    axis = local.to_3x3() @ Vector((0, 0, 0.5))
    assert abs(axis.length - 0.28) < 0.00001 and abs(axis.normalized().y) > 0.99999
    points = [local @ vertex.co for vertex in obj.data.vertices]
    for point in points:
        assert abs(point.x) <= dims.x * 0.5
        assert abs(point.y) <= dims.y * 0.5
        assert floor <= point.z <= ceiling
    bottom = min(bottom, min(point.z for point in points))
    top = max(top, max(point.z for point in points))
assert bottom - floor < 0.001, bottom - floor
assert top > dims.z * 0.5, top

bpy.data.objects.remove(preview.preview, do_unlink=True)
bpy.data.meshes.remove(preview.preview_mesh)
bpy.data.node_groups.remove(preview.preview_tree)
for obj in preview.preview_sources:
    bpy.data.objects.remove(obj)
settings.stack_mode = 'PHYSICS'
settings.fill_height = 1.55
settings.frames = 40
settings.substeps = 8
volume.scale.x = 0.35
volume.scale.z = 0.24
result, _ = addon["main"](bpy.context, settings)
bpy.context.view_layer.update()
frame, dims = addon["volume_geometry"](volume)
inverse = frame.inverted()
top = max((inverse @ obj.matrix_world @ vertex.co).z
          for obj in result.objects for vertex in obj.data.vertices)
assert dims.z * 0.5 < top <= dims.z * (settings.fill_height - 0.5) + 0.001, top

addon["unregister"]()
print(f"PRODUCT PREVIEW PASS on Blender {bpy.app.version_string}: {counts}")
