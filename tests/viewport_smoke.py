import runpy
import os
from pathlib import Path
import traceback
from tempfile import TemporaryDirectory

import bpy
from mathutils import Vector

addon = runpy.run_path(str(Path(__file__).resolve().parents[1] / "__init__.py"))
addon["register"]()
editor_type = addon["FWSHELF_OT_adjust_height"]
stage = 0

def get_editor():
    return editor_type.sessions[scene.as_pointer()]

def event(kind, value, point):
    window.event_simulate(type=kind, value=value, x=region.x + round(point.x), y=region.y + round(point.y))

def on_rail(height):
    bottom, top, w0, w1, _ = get_editor().rail
    return bottom.lerp(top, height * w1 / ((1 - height) * w0 + height * w1))

def assert_preview_geometry(editor):
    tree = editor.preview_tree
    output = next(node for node in tree.nodes if node.bl_idname == 'NodeGroupOutput')
    socket = output.inputs["Geometry"]
    instanced = socket.links[0].from_socket
    realize = tree.nodes.new('GeometryNodeRealizeInstances')
    try:
        tree.links.new(instanced, realize.inputs["Geometry"])
        tree.links.new(realize.outputs["Geometry"], socket)
        editor.preview.update_tag()
        depsgraph = bpy.context.evaluated_depsgraph_get()
        actual = len(editor.preview.evaluated_get(depsgraph).data.vertices)
        variants = editor.preview.data.attributes["fw_variant"].data
        expected = sum(len(editor.setups[variants[i].value][0].data.vertices) for i in range(editor.count))
        assert actual == expected, (actual, expected)
    finally:
        tree.nodes.remove(realize)
        tree.links.new(instanced, socket)
        editor.preview.update_tag()


def tick():
    global stage, window, area, region, scene, settings, full_count, half_count, baked, preview_id
    try:
        if stage == 0:
            window = bpy.context.window
            area = next(a for a in window.screen.areas if a.type == 'VIEW_3D')
            region = next(r for r in area.regions if r.type == 'WINDOW')
            sidebar = next(r for r in area.regions if r.type == 'UI')
            scene = window.scene
            settings = scene.fw_shelf_settings
            bpy.data.objects.remove(scene.objects["Cube"], do_unlink=True)
            with bpy.context.temp_override(window=window, area=area, region=region):
                bpy.ops.fw_shelf.make_volume()
                settings.volume_object.scale.x *= 2
                settings.volume_object.scale.z *= 2
                bpy.context.view_layer.update()
                bpy.ops.view3d.view_selected(use_all_regions=False)
                source = bpy.data.collections.new("Brennholz")
                scene.collection.children.link(source)
                for sides in (5, 11):
                    bpy.ops.mesh.primitive_cylinder_add(vertices=sides, radius=0.1, depth=0.5, location=(8, 0, 0))
                    obj = bpy.context.object
                    obj.users_collection[0].objects.unlink(obj)
                    source.objects.link(obj)
                    if sides == 11:
                        obj.modifiers.new("Vorschau muss Originalmesh verwenden", 'SUBSURF')
                settings.source_collection = source
                settings.length_mode = 'FIXED'
                settings.log_length = 0.28
                settings.frames = 45
                settings.substeps = 8
                settings.manual_count = 12
                assert bpy.ops.fw_shelf.generate() == {'FINISHED'}
                settings.manual_count = 0
            area.spaces.active.show_region_ui = True
            with bpy.context.temp_override(window=window, area=area, region=sidebar):
                assert bpy.ops.fw_shelf.adjust_height('INVOKE_DEFAULT') == {'RUNNING_MODAL'}
        elif stage == 1:
            editor = get_editor()
            assert editor.rail is not None
            full_count = editor.count
            assert full_count > 450 and editor.previous_layer.hide_viewport
            assert len(editor.preview.data.vertices) == full_count
            assert_preview_geometry(editor)
            preview_id = editor.preview.as_pointer()
            start = on_rail(1.0)
            event('MOUSEMOVE', 'NOTHING', start)
            event('LEFTMOUSE', 'PRESS', start)
            event('MOUSEMOVE', 'NOTHING', on_rail(0.50))
        elif stage == 2:
            editor = get_editor()
            half_count = editor.count
            assert abs(editor.height - 0.50) < 0.015, editor.height
            assert 5 < half_count < full_count, (half_count, full_count)
            assert editor.preview.as_pointer() == preview_id
            assert editor.dragging and settings.fill_height == 1.0
            assert_preview_geometry(editor)
            event('MOUSEMOVE', 'NOTHING', on_rail(0.83))
        elif stage == 3:
            editor = get_editor()
            assert half_count < editor.count <= full_count
            assert editor.preview.as_pointer() == preview_id
            event('MOUSEMOVE', 'NOTHING', on_rail(0.50))
            event('LEFTMOUSE', 'RELEASE', on_rail(0.50))
        elif stage == 4:
            editor = get_editor()
            assert editor.count == half_count
            _, _, _, _, (x1, y1, x2, y2) = editor.rail
            button = Vector(((x1 + x2) / 2, (y1 + y2) / 2))
            event('MOUSEMOVE', 'NOTHING', button)
            event('LEFTMOUSE', 'PRESS', button)
            event('LEFTMOUSE', 'RELEASE', button)
        elif stage == 5:
            assert not editor_type.sessions
            assert abs(settings.fill_height - 0.50) < 0.015
            assert bpy.data.objects.get("Firewood Live-Vorschau") is None
            baked = addon["find_stack"](scene, settings.volume_object)
            assert len(baked.objects) >= half_count * 0.97
            assert all(obj.parent == settings.volume_object for obj in baked.objects)
            bpy.context.view_layer.update()
            assert all(abs((obj.matrix_world.to_3x3() @ Vector((0, 0, 0.5))).length - 0.28) < 0.00001
                       for obj in baked.objects)
            with bpy.context.temp_override(window=window, area=area, region=region):
                assert bpy.ops.fw_shelf.adjust_height('INVOKE_DEFAULT') == {'RUNNING_MODAL'}
        elif stage == 6:
            event('MOUSEMOVE', 'NOTHING', on_rail(0.5))
            event('LEFTMOUSE', 'PRESS', on_rail(0.5))
            event('MOUSEMOVE', 'NOTHING', on_rail(0.0))
        elif stage == 7:
            assert get_editor().count == 0
            event('ESC', 'PRESS', on_rail(0))
        elif stage == 8:
            assert not editor_type.sessions
            assert abs(settings.fill_height - 0.50) < 0.015
            assert addon["find_stack"](scene, settings.volume_object) == baked
            layer = bpy.context.view_layer.layer_collection.children.get(baked.name)
            assert not layer.hide_viewport and bpy.data.objects.get("Firewood Live-Vorschau") is None
            with bpy.context.temp_override(window=window, area=area, region=region):
                assert bpy.ops.fw_shelf.adjust_height('INVOKE_DEFAULT') == {'RUNNING_MODAL'}
        elif stage == 9:
            assert get_editor().preview is not None
            with TemporaryDirectory(prefix="firewood-save-") as directory:
                assert bpy.ops.wm.save_as_mainfile(filepath=str(Path(directory) / "test.blend"), copy=True) == {'FINISHED'}
            assert not editor_type.sessions and bpy.data.objects.get("Firewood Live-Vorschau") is None
            assert not bpy.context.view_layer.layer_collection.children.get(baked.name).hide_viewport
            print(f"GUI LIVE PASS: {full_count} -> {half_count} logs while dragging, then raised, applied and cancelled", flush=True)
            bpy.ops.wm.quit_blender()
            return None
        stage += 1
        return 0.9
    except Exception:
        traceback.print_exc()
        os._exit(1)

bpy.app.timers.register(tick, first_interval=1.5)
