# -*- coding: utf-8 -*-
bl_info = {
    "name": "Firewood Shelf Filler",
    "author": "OpenAI",
    "version": (1, 3, 0),
    "blender": (4, 2, 0),
    "location": "3D Viewport > N > Firewood",
    "description": "Kleinere Holzscheite physikalisch dicht stapeln und als statische Instanzen einfrieren",
    "category": "Object",
}

"""Firewood Shelf Filler -- Blender 4.x / 5.x.

Bedienung: Quell-Collection mit einzelnen Holzscheiten waehlen; einen Cube
als Innenraum des Regalfachs waehlen; 'Stapel erzeugen' klicken.
Lokale X-Achse des Cubes = Breite, Y = Tiefe, Z = Hoehe.
Das Volumen muss aufrecht stehen, darf aber um Z gedreht sein.

Die Simulation nutzt vereinfachte Convex-Hulls, die danach entfernt werden.
Das Ergebnis ist statisch und verwendet die Mesh-Daten der Quellen gemeinsam.
Die Quelle bleibt unveraendert; bei Quellen mit Modifiers sollten diese
vorher auf einer Kopie angewendet werden. Materialien im Mesh bleiben erhalten.

Die Simulation läuft in einer temporären Szene und berührt die Physik der
aktiven Szene nicht.
"""

import bpy
import bmesh
import math
import random

import blf
import gpu
from gpu_extras.batch import batch_for_shader
from bpy_extras.view3d_utils import location_3d_to_region_2d
from mathutils import Matrix, Vector, Euler
from bpy.props import BoolProperty, FloatProperty, IntProperty, PointerProperty

SIM_SOLVER_ITERATIONS = 16
COLLISION_MARGIN = 0.0008
WALL_THICKNESS = 0.04
FRICTION = 0.80
PROXY_MAX_VERTICES = 64
LENGTH_VARIATION = (0.90, 1.06)
DIAMETER_VARIATION = (0.88, 1.06)
GENERATED_COLLECTION = "fw_shelf_generated_collection"
STACK_VOLUME = "fw_shelf_volume_object"

def log(message):
    print("[Firewood Filler] " + str(message), flush=True)


def collection_contains_object(collection, obj):
    """Collection membership by name: compatible with Blender 4.x and 5.x.

    bpy_prop_collection.__contains__ expects a string in Blender 5.3.
    Compare the returned object too, rather than relying on a name alone.
    """
    return obj is not None and collection.all_objects.get(obj.name) == obj


def volume_geometry(volume):
    """Return the centered, upright coordinate frame and dimensions of the shelf."""
    up = volume.matrix_world.to_quaternion() @ Vector((0, 0, 1))
    if up.dot(Vector((0, 0, 1))) < 0.985:
        raise RuntimeError("Der Regal-Innenraum muss aufrecht stehen (lokale Z-Achse nach oben).")

    corners = tuple(volume.bound_box)
    low = Vector(tuple(min(v[i] for v in corners) for i in range(3)))
    high = Vector(tuple(max(v[i] for v in corners) for i in range(3)))
    scales = volume.matrix_world.to_scale()
    dims = Vector(tuple((high[i] - low[i]) * abs(scales[i]) for i in range(3)))
    if min(dims) < 0.11:
        raise RuntimeError("Der Regal-Innenraum muss in jeder Achse mindestens 0,11 Blender-Einheiten messen.")

    frame = volume.matrix_world.to_quaternion().to_matrix().to_4x4()
    frame.translation = volume.matrix_world @ ((low + high) * 0.5)
    return frame, dims


def fill_geometry(frame, dims, height):
    """Keep the floor fixed and move the simulation ceiling to the chosen height."""
    filled = dims.copy()
    filled.z *= height
    offset = Vector((0, 0, (filled.z - dims.z) * 0.5))
    return frame @ Matrix.Translation(offset), filled


def source_variants(collection):
    return [obj for obj in collection.all_objects if obj.type == 'MESH'
            and len(obj.data.vertices) >= 8 and not obj.get('firewood_generated')]


def check_input(scene, settings):
    source = settings.source_collection
    volume = settings.volume_object
    if source is None:
        raise RuntimeError("Bitte zuerst eine Holzscheit-Collection auswählen.")
    if volume is None or volume.type != 'MESH' or scene.objects.get(volume.name) != volume:
        raise RuntimeError("Bitte einen Mesh-Würfel der aktiven Szene als Regal-Innenraum wählen.")
    if collection_contains_object(source, volume):
        raise RuntimeError("Der Regal-Innenraum darf nicht in der Quell-Collection liegen.")
    variants = source_variants(source)
    if not variants:
        raise RuntimeError("Keine gültigen Holzscheit-Meshes in der Quell-Collection.")
    frame, dims = fill_geometry(*volume_geometry(volume), settings.fill_height)
    return variants, frame, dims

def new_collection(name, scene):
    collection = bpy.data.collections.new(name)
    scene.collection.children.link(collection)
    return collection


def find_stack(scene, volume):
    legacy = None
    for collection in scene.collection.children:
        if not collection.get(GENERATED_COLLECTION):
            continue
        if STACK_VOLUME in collection:
            if collection[STACK_VOLUME] == volume:
                return collection
        elif collection.get("fw_shelf_volume") == volume.name:
            legacy = collection
    return legacy


def remove_stack(scene, collection):
    # A collection shared with another scene is only unlinked from this scene.
    if collection.users > 1:
        scene.collection.children.unlink(collection)
        return
    for obj in tuple(collection.objects):
        if obj.get("firewood_generated") and len(obj.users_collection) == 1:
            bpy.data.objects.remove(obj, do_unlink=True)
    bpy.data.collections.remove(collection)

def make_box_mesh(name, dims):
    x, y, z = (d * 0.5 for d in dims)
    verts = [(-x, -y, -z), ( x, -y, -z), ( x, y, -z), (-x, y, -z),
             (-x, -y,  z), ( x, -y,  z), ( x, y,  z), (-x, y,  z)]
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    return mesh


def add_rigidbody(obj, kind, shape):
    with bpy.context.temp_override(object=obj, active_object=obj,
                                   selected_objects=[obj], selected_editable_objects=[obj]):
        status = bpy.ops.rigidbody.object_add(type=kind)
    if status != {'FINISHED'} or obj.rigid_body is None:
        raise RuntimeError(f"Physik-Körper konnte nicht erstellt werden: {obj.name}")
    obj.rigid_body.collision_shape = shape
    obj.rigid_body.use_margin = True
    obj.rigid_body.collision_margin = COLLISION_MARGIN
    obj.rigid_body.friction = FRICTION
    obj.rigid_body.restitution = 0.0
    return obj.rigid_body


def create_walls(temp, frame, dims, meshes):
    w, d, h = dims
    t = WALL_THICKNESS
    specs = [
        ("bottom", (0, 0, -h*0.5-t*0.5), (w+2*t, d+2*t, t)),
        ("left",   (-w*0.5-t*0.5, 0, 0), (t, d+2*t, h+2*t)),
        ("right",  ( w*0.5+t*0.5, 0, 0), (t, d+2*t, h+2*t)),
        ("back",   (0,  d*0.5+t*0.5, 0), (w+2*t, t, h+2*t)),
        ("front",  (0, -d*0.5-t*0.5, 0), (w+2*t, t, h+2*t)),
        # Die Deckelbegrenzung verhindert, dass Scheite beim Stapeln
        # aus dem vorgesehenen Regalfach springen.
        ("top",    (0, 0, h*0.5+t*0.5), (w+2*t, d+2*t, t)),
    ]
    for label, p, size in specs:
        mesh = make_box_mesh("__FW_WallMesh_" + label, size)
        meshes.append(mesh)
        obj = bpy.data.objects.new("__FW_wall_" + label, mesh)
        temp.objects.link(obj)
        obj.matrix_world = frame @ Matrix.Translation(Vector(p))
        add_rigidbody(obj, 'PASSIVE', 'BOX')
        obj.hide_render = True
        obj.display_type = 'WIRE'


def normalize_variant(source, target_length, target_diameter):
    low = Vector(tuple(min(v[i] for v in source.bound_box) for i in range(3)))
    high = Vector(tuple(max(v[i] for v in source.bound_box) for i in range(3)))
    center = (low + high) * 0.5
    extent = high - low
    source_scale = source.matrix_world.to_scale()
    axis = max(range(3), key=lambda i: extent[i] * abs(source_scale[i]))
    direction = Vector(tuple(float(i == axis) for i in range(3)))
    cross_axes = tuple(i for i in range(3) if i != axis)
    radius = max(math.hypot(*((vertex.co[i] - center[i]) * source_scale[i]
                             for i in cross_axes)) for vertex in source.data.vertices)
    length = extent[axis] * abs(source_scale[axis])
    if min(radius, length, *(abs(v) for v in source_scale)) < 1e-8:
        raise RuntimeError(f"'{source.name}' hat keine gültigen Abmessungen.")
    factors = [factor * target_diameter / (2 * radius) for factor in source_scale]
    factors[axis] = target_length / extent[axis]
    align = direction.rotation_difference(Vector((0, 1, 0))).to_matrix()
    return align.to_4x4() @ Matrix.Diagonal((*factors, 1.0)) @ Matrix.Translation(-center)

def create_proxy_mesh(source, transform, index, seed):
    # Erzeuge pro Quellmesh nur EINE vereinfachte Konvexhuelle.
    # Alle simulierten Kopien verweisen auf diese Proxy-Mesh-Daten.
    vertices = source.data.vertices
    n = len(vertices)
    rand = random.Random(seed + 1000 + index)
    chosen = set(rand.sample(range(n), min(PROXY_MAX_VERTICES, n)))
    # Extreme fuer eine robuste Huellenform explizit einbeziehen.
    for axis in range(3):
        chosen.add(min(range(n), key=lambda k: vertices[k].co[axis]))
        chosen.add(max(range(n), key=lambda k: vertices[k].co[axis]))

    points = [transform @ vertices[k].co for k in sorted(chosen)]
    mesh = bpy.data.meshes.new("__FW_ConvexHull_%03d" % index)
    bm = bmesh.new()
    try:
        for p in points:
            bm.verts.new(p)
        bm.verts.ensure_lookup_table()
        result = bmesh.ops.convex_hull(bm, input=list(bm.verts), use_existing_faces=False)
        leftovers = set(result.get('geom_interior', [])) | set(result.get('geom_unused', []))
        unused_vertices = [item for item in leftovers if isinstance(item, bmesh.types.BMVert)]
        if unused_vertices:
            bmesh.ops.delete(bm, geom=unused_vertices, context='VERTS')
        if not bm.faces:
            raise ValueError("Konvexhuelle ohne Flaechen")
        bm.to_mesh(mesh)
        mesh.update()
    except Exception:
        # Fallback (auch bei stark degenerierten Scans).
        log(f"{source.name}: Hull-Fehler; benutze Box-Proxy")
        bpy.data.meshes.remove(mesh)
        low = [min(v[i] for v in points) for i in range(3)]
        high = [max(v[i] for v in points) for i in range(3)]
        extent = Vector([max(0.005, high[i]-low[i]) for i in range(3)])
        mesh = make_box_mesh("__FW_BoxProxy_%03d" % index, extent)
    finally:
        bm.free()
    return mesh


def slot_grid(dims, diameter):
    # Allow for the largest variant, its roll, and Bullet's collision margin.
    pitch = diameter * 1.04 * DIAMETER_VARIATION[1] + 2 * COLLISION_MARGIN
    inset = pitch * 0.5 + COLLISION_MARGIN
    width = dims.x - 2 * inset
    even = max(0, math.floor(width / pitch) + 1)
    odd = max(0, math.floor((width - pitch * 0.5) / pitch) + 1)
    rows = max(0, math.floor((dims.z - 2 * inset) / pitch) + 1)
    capacity = (rows // 2) * (even + odd) + (rows % 2) * even
    return pitch, inset, even, odd, capacity


def planned_count(settings, capacity):
    if settings.manual_count:
        return min(settings.manual_count, capacity)
    return min(settings.max_auto_count, max(1, round(capacity * settings.fill)), capacity)


def create_slots(dims, settings):
    pitch, inset, even, odd, capacity = slot_grid(dims, settings.diameter)
    amount = planned_count(settings, capacity)
    # Keep the lower rows filled. Only sample positions in the last partial row.
    slots = []
    row = 0
    while len(slots) < amount:
        columns = odd if row % 2 else even
        take = min(columns, amount - len(slots))
        if not take:
            row += 1
            continue
        rng = random.Random(settings.seed + row)
        first = rng.randrange(columns)
        step = rng.randrange(1, columns + 1)
        while math.gcd(step, columns) != 1:
            step += 1
        for i in range(take):
            column = (first + step * i) % columns
            x = -dims.x * 0.5 + inset + (column + (row % 2) * 0.5) * pitch
            z = -dims.z * 0.5 + inset + row * pitch
            slots.append((x, z))
        row += 1
    return slots


def prepare_variants(variants, dims, settings):
    rng = random.Random(settings.seed)
    target_len = dims.y * settings.depth_factor
    max_length = (dims.y - 4 * COLLISION_MARGIN) / DIAMETER_VARIATION[1]
    setups = []
    for source in variants:
        if source.modifiers:
            log(f"{source.name}: Für Instanzen wird das Mesh ohne Modifier verwendet.")
        length = min(target_len * rng.uniform(*LENGTH_VARIATION), max_length)
        diameter = settings.diameter * rng.uniform(0.93, 1.04)
        setups.append((source, normalize_variant(source, length, diameter)))
    return setups


def placements(frame, dims, slots, settings, variant_count):
    rng = random.Random(settings.seed)
    max_length = (dims.y - 4 * COLLISION_MARGIN) / DIAMETER_VARIATION[1]
    for x, z in slots:
        variant = rng.randrange(variant_count)
        angle = Euler((0, rng.uniform(-math.pi, math.pi), 0), 'XYZ')
        scale = rng.uniform(*DIAMETER_VARIATION)
        y_room = max(0, min(0.010, (dims.y - max_length * scale) * 0.5 - COLLISION_MARGIN))
        p = Vector((x, rng.uniform(-y_room, y_room), z))
        yield variant, (frame @ Matrix.Translation(p) @ angle.to_matrix().to_4x4()
                        @ Matrix.Scale(scale, 4))


def create_proxies(temp, frame, dims, variants, slots, settings, meshes):
    setups = prepare_variants(variants, dims, settings)
    proxies = []
    for index, (source, transform) in enumerate(setups):
        mesh = create_proxy_mesh(source, transform, index, settings.seed)
        meshes.append(mesh)
        proxies.append(mesh)
    log(f"{len(variants)} Varianten, {len(slots)} Startpositionen.")

    bodies = []
    for i, (variant, matrix) in enumerate(placements(frame, dims, slots, settings, len(setups))):
        obj = bpy.data.objects.new(f"__FW_log_{i:04d}", proxies[variant])
        temp.objects.link(obj)
        obj.matrix_world = matrix
        rb = add_rigidbody(obj, 'ACTIVE', 'CONVEX_HULL')

        rb.mass = max(0.05, (settings.diameter / 0.095)**2)
        rb.linear_damping = 0.30
        rb.angular_damping = 0.43
        rb.use_deactivation = True
        obj.hide_render = True
        obj.display_type = 'WIRE'
        bodies.append((obj, setups[variant]))
    return bodies


def simulate_and_snapshot(scene, bodies, settings, progress=None):
    world = scene.rigidbody_world
    if world is None:
        raise RuntimeError("Rigid Body World konnte nicht erzeugt werden.")
    world.substeps_per_frame = settings.substeps
    world.solver_iterations = SIM_SOLVER_ITERATIONS
    world.point_cache.frame_start = 1
    world.point_cache.frame_end = settings.frames

    scene.frame_set(1)
    bpy.context.view_layer.update()
    for frame_number in range(2, settings.frames + 1):
        scene.frame_set(frame_number)
        if progress is not None:
            progress(frame_number)
        if frame_number % 30 == 0 or frame_number == settings.frames:
            log(f"Physik: Frame {frame_number}/{settings.frames}")
    depsgraph = bpy.context.evaluated_depsgraph_get()
    snapshots = []
    for obj, setup in bodies:
        evaluated = obj.evaluated_get(depsgraph)
        snapshots.append((evaluated.matrix_world.copy(), setup))
    return snapshots


def inside_fill(mesh, matrix, half_dims):
    def inside(point):
        return all(abs(value) <= limit + 0.001 for value, limit in zip(point, half_dims))

    # The bounding box handles most logs cheaply. At the wall, check the actual mesh.
    if all(inside(matrix @ Vector(corner)) for corner in mesh.bound_box):
        return True
    return all(inside(matrix @ vertex.co) for vertex in mesh.data.vertices)


def link_log(collection, source, name, matrix):
    obj = bpy.data.objects.new(name, source.data)
    collection.objects.link(obj)
    try:
        for src_slot, dst_slot in zip(source.material_slots, obj.material_slots):
            if src_slot.link == 'OBJECT':
                dst_slot.link = 'OBJECT'
                dst_slot.material = src_slot.material
        obj.color = source.color
        obj.matrix_world = matrix
    except Exception:
        bpy.data.objects.remove(obj, do_unlink=True)
        raise
    return obj


def create_final(scene, volume, snapshots):
    previous = find_stack(scene, volume)
    name = "FW_Stack_" + volume.name[:45]
    final = new_collection(name, scene)
    final[GENERATED_COLLECTION] = True
    final[STACK_VOLUME] = volume
    try:
        for i, (matrix, (source, transform)) in enumerate(snapshots):
            obj = link_log(final, source, f"FW_Log_{i:04d}_{source.name}", matrix @ transform)
            obj["firewood_generated"] = True
            obj["firewood_variant"] = source.name
    except Exception:
        remove_stack(scene, final)
        raise
    if previous is not None:
        remove_stack(scene, previous)
        final.name = name
    return final


def main(context, settings, progress=None):
    scene = context.scene
    variants, frame, dims = check_input(scene, settings)
    slots = create_slots(dims, settings)
    if not slots:
        raise RuntimeError("Die Füllhöhe oder Regalbreite reicht für die eingestellte Scheit-Dicke nicht aus.")
    inverse = frame.inverted()
    meshes = []
    simulation = bpy.data.scenes.new("Firewood Physics")
    try:
        with context.temp_override(scene=simulation, view_layer=simulation.view_layers[0]):
            try:
                simulation.frame_end = settings.frames
                bpy.ops.rigidbody.world_add()
                create_walls(simulation.collection, frame, dims, meshes)
                bodies = create_proxies(simulation.collection, frame, dims, variants, slots, settings, meshes)
                snapshots = simulate_and_snapshot(simulation, bodies, settings, progress=progress)
            finally:
                if simulation.rigidbody_world is not None:
                    body_collection = simulation.rigidbody_world.collection
                    bpy.ops.rigidbody.world_remove()
                    if body_collection is not None:
                        bpy.data.collections.remove(body_collection)
                for obj in tuple(simulation.objects):
                    bpy.data.objects.remove(obj, do_unlink=True)
    finally:
        bpy.data.scenes.remove(simulation)
        for mesh in meshes:
            bpy.data.meshes.remove(mesh)

    valid = [(matrix, setup) for matrix, setup in snapshots
             if inside_fill(setup[0], inverse @ matrix @ setup[1], dims * 0.5)]
    if not valid:
        raise RuntimeError("Kein Scheit blieb im Füllbereich. Prüfe Füllhöhe, Dicke und Simulationsdauer.")
    return create_final(scene, settings.volume_object, valid), len(snapshots) - len(valid)

class FWShelfSettings(bpy.types.PropertyGroup):
    source_collection: PointerProperty(
        name="Holzscheite", type=bpy.types.Collection,
        description="Collection mit den urspruenglichen Holzscheit-Meshes",
    )
    volume_object: PointerProperty(
        name="Regal-Innenraum", type=bpy.types.Object,
        description="Cube, der den freien Platz im Regal ausfuellt (X=Breite, Y=Tiefe, Z=Hoehe)",
    )
    fill: FloatProperty(
        name="Rasterbelegung", description="Anteil der Startpositionen innerhalb der gewählten Füllhöhe",
        default=0.97, min=0.10, max=1.0, subtype='FACTOR',
    )
    fill_height: FloatProperty(
        name="Füllhöhe", description="Obergrenze des Stapels, gemessen vom Boden des Innenraums",
        default=1.0, min=0.0, max=1.0, subtype='FACTOR',
    )
    manual_count: IntProperty(
        name="Anzahl (0 = Auto)", description="0 = passend zur Regalbreite/-hoehe berechnen",
        default=0, min=0, max=2000,
    )
    diameter: FloatProperty(
        name="Scheit-Dicke", description="Ungefaehrer Durchmesser in Blender-Laengeneinheiten (bei Unit Scale 1: Meter)",
        default=0.055, min=0.015, max=1.0, precision=3, subtype='DISTANCE',
    )
    max_auto_count: IntProperty(
        name="Max. Scheite (Auto)",
        description="Sicherheitslimit fuer automatisch erzeugte Physik-Objekte; hoeher ist langsamer",
        default=450, min=20, max=2000,
    )
    depth_factor: FloatProperty(
        name="Laenge / Regaltiefe", description="Typische Scheitlaenge relativ zur Regaltiefe",
        default=0.76, min=0.40, max=0.90, subtype='FACTOR',
    )
    seed: IntProperty(
        name="Zufalls-Seed", default=42, min=0, max=999999,
    )
    frames: IntProperty(
        name="Simulations-Frames", description="Hoeher = genauer, aber langsamer",
        default=220, min=40, max=800,
    )
    substeps: IntProperty(
        name="Physik-Substeps", description="Berechnungsschritte pro Frame",
        default=14, min=5, max=100,
    )


class FWSHELF_OT_small_preset(bpy.types.Operator):
    bl_idname = "fw_shelf.small_preset"
    bl_label = "Klein + viele (Physik)"
    bl_description = "Stellt kleinere Holzscheite und eine groessere automatische Anzahl ein"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        p = context.scene.fw_shelf_settings
        p.manual_count = 0
        p.diameter = 0.055
        p.fill = 0.97
        p.max_auto_count = 450
        p.frames = 220
        p.substeps = 14
        self.report({'INFO'}, "Physik-Preset: 5,5 cm, 97 %, bis 450 Holzscheite.")
        return {'FINISHED'}


class FWSHELF_OT_make_volume(bpy.types.Operator):
    bl_idname = "fw_shelf.make_volume"
    bl_label = "Fuellwuerfel erstellen"
    bl_description = "Erzeugt einen anpassbaren Cube als Begrenzung des Regalfachs"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        if context.mode != 'OBJECT':
            self.report({'ERROR'}, "Bitte zuerst in den Object Mode wechseln.")
            return {'CANCELLED'}
        bpy.ops.mesh.primitive_cube_add(size=1.0, location=context.scene.cursor.location)
        obj = context.object
        obj.name = "FIREWOOD_VOLUME"
        obj.dimensions = (1.0, 0.36, 0.60)
        obj.display_type = 'WIRE'
        obj.hide_render = True
        context.scene.fw_shelf_settings.volume_object = obj
        self.report({'INFO'}, "Wuerfel an das freie Regalfach anpassen (X Breite, Y Tiefe, Z Hoehe).")
        return {'FINISHED'}


class FWSHELF_OT_generate(bpy.types.Operator):
    bl_idname = "fw_shelf.generate"
    bl_label = "Holzstapel erzeugen"
    bl_description = "Simuliert den Holzstapel und erstellt danach statische, datengebundene Objekte"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        if context.mode != 'OBJECT':
            self.report({'ERROR'}, "Bitte zuerst in den Objektmodus wechseln.")
            return {'CANCELLED'}
        settings = context.scene.fw_shelf_settings
        wm = context.window_manager
        wm.progress_begin(1, settings.frames)
        try:
            result, discarded = main(context, settings, progress=wm.progress_update)
        except (RuntimeError, ValueError) as exc:
            self.report({'ERROR'}, f"Firewood: {exc}")
            return {'CANCELLED'}
        finally:
            wm.progress_end()
        message = f"{len(result.objects)} Holzscheite in '{result.name}' erzeugt."
        if discarded:
            message += f" {discarded} Scheite lagen außerhalb der Füllhöhe oder des Regalfachs."
        self.report({'WARNING'} if discarded else {'INFO'}, message)
        return {'FINISHED'}


class FWSHELF_OT_clear(bpy.types.Operator):
    bl_idname = "fw_shelf.clear"
    bl_label = "Erzeugten Stapel entfernen"
    bl_description = "Entfernt nur den von diesem Add-on erzeugten Stapel fuer das gewaehlte Volumen"
    bl_options = {'REGISTER', 'UNDO'}

    def invoke(self, context, event):
        return context.window_manager.invoke_confirm(self, event)

    def execute(self, context):
        vol = context.scene.fw_shelf_settings.volume_object
        if vol is None:
            self.report({'WARNING'}, "Bitte zuerst Regal-Innenraum auswaehlen.")
            return {'CANCELLED'}
        collection = find_stack(context.scene, vol)
        if collection is None:
            self.report({'WARNING'}, "Kein vom Add-on erzeugter Stapel fuer dieses Regal vorhanden.")
            return {'CANCELLED'}
        count = len(collection.objects)
        remove_stack(context.scene, collection)
        self.report({'INFO'}, f"{count} generierte Scheite entfernt; Originale behalten.")
        return {'FINISHED'}


class FWSHELF_OT_adjust_height(bpy.types.Operator):
    bl_idname = "fw_shelf.adjust_height"
    bl_label = "Füllhöhe im Viewport einstellen"
    bl_description = "Die Füllhöhe ziehen und die neue Oberkante direkt im Regal sehen"
    bl_options = {'UNDO'}

    sessions = {}
    accent = (0.28, 0.83, 0.73, 1.0)

    @classmethod
    def poll(cls, context):
        return context.area is not None and context.area.type == 'VIEW_3D' and context.mode == 'OBJECT'

    def invoke(self, context, event):
        settings = context.scene.fw_shelf_settings
        volume = settings.volume_object
        if volume is None or volume.type != 'MESH' or context.scene.objects.get(volume.name) != volume:
            self.report({'WARNING'}, "Bitte zuerst den Regal-Innenraum wählen.")
            return {'CANCELLED'}
        try:
            self.frame, self.dims = volume_geometry(volume)
        except RuntimeError as exc:
            self.report({'WARNING'}, str(exc))
            return {'CANCELLED'}
        self.scene_id = context.scene.as_pointer()
        if self.scene_id in self.sessions:
            self.report({'WARNING'}, "Der Höhenregler ist für diese Szene bereits geöffnet.")
            return {'CANCELLED'}
        region = context.region
        if region.type != 'WINDOW':
            region = next((r for r in context.area.regions if r.type == 'WINDOW'), None)
        if region is None or region.width < 200 or region.height < 125:
            self.report({'WARNING'}, "Für den Höhenregler den 3D-Viewport vergrößern.")
            return {'CANCELLED'}

        self.window = context.window
        self.wm = context.window_manager
        self.area_id = context.area.as_pointer()
        self.region_id = region.as_pointer()
        self.volume = volume
        self.settings = settings
        self.height = settings.fill_height
        self.ui_scale = context.preferences.system.ui_scale
        self.scene = context.scene
        self.setups = []
        source = settings.source_collection
        if source is not None and not collection_contains_object(source, volume):
            try:
                self.setups = prepare_variants(source_variants(source), self.dims, settings)
            except RuntimeError as exc:
                self.report({'WARNING'}, str(exc))
        self.preview = None
        self.preview_logs = []
        self.previous_layer = None
        self.previous_hidden = False
        self.rail = None
        self.count = None
        self.dragging = False
        self.handlers = []
        self.timer = None
        self.sessions[self.scene_id] = self
        try:
            self.handlers.append(bpy.types.SpaceView3D.draw_handler_add(self.draw_world, (), 'WINDOW', 'POST_VIEW'))
            self.handlers.append(bpy.types.SpaceView3D.draw_handler_add(self.draw_hud, (), 'WINDOW', 'POST_PIXEL'))
            if self.setups:
                self.preview = new_collection("Firewood Live-Vorschau", self.scene)
                self.preview.hide_render = True
                previous = find_stack(self.scene, volume)
                if previous is not None:
                    self.previous_layer = context.view_layer.layer_collection.children.get(previous.name)
                    if self.previous_layer is not None:
                        self.previous_hidden = self.previous_layer.hide_viewport
                        self.previous_layer.hide_viewport = True
            self.update_preview()
            self.timer = self.wm.event_timer_add(0.06, window=self.window)
            self.wm.modal_handler_add(self)
        except Exception:
            self.close()
            raise
        context.area.tag_redraw()
        return {'RUNNING_MODAL'}

    def area(self):
        return next((a for a in self.window.screen.areas
                     if a.as_pointer() == self.area_id and a.type == 'VIEW_3D'), None)

    def close(self):
        if self.sessions.pop(self.scene_id, None) is None:
            return
        for handler in self.handlers:
            bpy.types.SpaceView3D.draw_handler_remove(handler, 'WINDOW')
        if self.timer is not None:
            self.wm.event_timer_remove(self.timer)
        for obj in self.preview_logs:
            bpy.data.objects.remove(obj, do_unlink=True)
        if self.preview is not None:
            bpy.data.collections.remove(self.preview)
        try:
            if self.previous_layer is not None:
                self.previous_layer.hide_viewport = self.previous_hidden
            area = self.area()
            if area is not None:
                area.tag_redraw()
        except ReferenceError:
            pass

    def cancel(self, context):
        self.close()

    def update_preview(self):
        frame, dims = fill_geometry(self.frame, self.dims, self.height)
        count = planned_count(self.settings, slot_grid(dims, self.settings.diameter)[-1])
        if count == self.count:
            return
        self.count = count
        if self.preview is None:
            return
        while len(self.preview_logs) > count:
            bpy.data.objects.remove(self.preview_logs.pop(), do_unlink=True)
        if len(self.preview_logs) == count:
            return
        slots = create_slots(dims, self.settings)
        for i, (variant, matrix) in enumerate(placements(frame, dims, slots, self.settings, len(self.setups))):
            if i >= len(self.preview_logs):
                source, transform = self.setups[variant]
                obj = link_log(self.preview, source, "Firewood Vorschau", matrix @ transform)
                obj.hide_select = True
                self.preview_logs.append(obj)

    def apply_height(self):
        self.settings.fill_height = self.height
        if not self.setups:
            self.close()
            return {'FINISHED'}
        if not self.count:
            self.close()
            previous = find_stack(self.scene, self.volume)
            if previous is not None:
                remove_stack(self.scene, previous)
            return {'FINISHED'}
        self.close()
        try:
            result, discarded = main(bpy.context, self.settings)
        except (RuntimeError, ValueError) as exc:
            self.report({'WARNING'}, f"Füllhöhe gespeichert. Stapel konnte nicht erzeugt werden: {exc}")
        else:
            message = f"{len(result.objects)} Holzscheite erzeugt."
            if discarded:
                message += f" {discarded} Scheite lagen außerhalb des Füllbereichs."
            self.report({'WARNING'} if discarded else {'INFO'}, message)
        return {'FINISHED'}

    def modal(self, context, event):
        if self.sessions.get(self.scene_id) is not self:
            return {'CANCELLED'}
        area = self.area()
        region = next((r for r in area.regions if r.as_pointer() == self.region_id), None) if area else None
        if (region is None or context.scene.as_pointer() != self.scene_id
                or self.settings.volume_object != self.volume or self.settings.volume_object is None):
            self.close()
            return {'CANCELLED'}

        if event.type == 'TIMER' and self.dragging:
            self.update_preview()
            area.tag_redraw()
        if event.value == 'PRESS':
            if event.type in {'ESC', 'RIGHTMOUSE'}:
                self.close()
                return {'CANCELLED'}
            if event.type in {'RET', 'NUMPAD_ENTER'}:
                self.update_preview()
                return self.apply_height()

        if self.rail is None:
            return {'PASS_THROUGH'}
        bottom, top, w0, w1, button = self.rail
        mouse = Vector((event.mouse_x - region.x, event.mouse_y - region.y))
        direction = top - bottom
        t = min(1.0, max(0.0, (mouse - bottom).dot(direction) / direction.length_squared))
        if event.type == 'LEFTMOUSE' and event.value == 'PRESS':
            if button[0] <= mouse.x <= button[2] and button[1] <= mouse.y <= button[3]:
                self.update_preview()
                return self.apply_height()
            if (mouse - bottom - t * direction).length <= 13 * self.ui_scale:
                self.dragging = True
        if self.dragging and event.type in {'MOUSEMOVE', 'LEFTMOUSE'}:
            self.height = t * w0 / ((1 - t) * w1 + t * w0)
            if event.type == 'LEFTMOUSE' and event.value == 'RELEASE':
                self.dragging = False
                self.update_preview()
            area.tag_redraw()
            return {'RUNNING_MODAL'}
        return {'PASS_THROUGH'}

    def active_region(self):
        context = bpy.context
        if (context.area is not None and context.region is not None
                and context.area.as_pointer() == self.area_id
                and context.region.as_pointer() == self.region_id):
            return context.region
        return None

    def project_rail(self, region, view):
        half_height = self.dims.z * 0.5
        front_y = -self.dims.y * 0.5
        start = self.frame @ Vector((0, front_y, -half_height))
        end = self.frame @ Vector((0, front_y, half_height))
        bottom = location_3d_to_region_2d(region, view, start)
        top = location_3d_to_region_2d(region, view, end)
        if bottom is None or top is None:
            return None
        if (top - bottom).length < 36 * self.ui_scale:
            # A top view foreshortens the shelf's Z axis.
            center = (top + bottom) * 0.5
            offset = Vector((0, 45 * self.ui_scale))
            return center - offset, center + offset, 1.0, 1.0
        w0 = (view.perspective_matrix @ Vector((*start, 1.0))).w
        w1 = (view.perspective_matrix @ Vector((*end, 1.0))).w
        return bottom, top, w0, w1

    @staticmethod
    def draw_batch(shader, mode, points, color, indices=None):
        batch = batch_for_shader(shader, mode, {"pos": points}, indices=indices)
        shader.bind()
        shader.uniform_float("color", color)
        batch.draw(shader)

    def draw_world(self):
        if self.active_region() is None:
            return
        x, y, z = self.dims * 0.5
        top = -z + self.dims.z * self.height
        corners = [self.frame @ Vector((i*x, j*y, h))
                   for h in (-z, top) for i, j in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        shader = gpu.shader.from_builtin('UNIFORM_COLOR')
        blend, depth, mask = gpu.state.blend_get(), gpu.state.depth_test_get(), gpu.state.depth_mask_get()
        try:
            gpu.state.blend_set('ALPHA')
            gpu.state.depth_test_set('NONE')
            gpu.state.depth_mask_set(False)
            self.draw_batch(shader, 'TRIS', corners[4:], (*self.accent[:3], 0.22), ((0, 1, 2), (0, 2, 3)))
            edges = ((0, 1), (1, 2), (2, 3), (3, 0), (0, 4), (1, 5), (2, 6), (3, 7))
            self.draw_batch(shader, 'LINES', corners, (*self.accent[:3], 0.58), edges)
            self.draw_batch(shader, 'LINE_LOOP', corners[4:], self.accent)
        finally:
            gpu.state.depth_mask_set(mask)
            gpu.state.depth_test_set(depth)
            gpu.state.blend_set(blend)

    def draw_hud(self):
        region = self.active_region()
        if region is None:
            return
        view = bpy.context.region_data
        projection = self.project_rail(region, view) if view is not None else None
        if projection is None:
            self.rail = None
            return
        bottom, top, w0, w1 = projection
        t = self.height * w1 / ((1 - self.height) * w0 + self.height * w1)
        knob = bottom.lerp(top, t)
        scale = self.ui_scale
        size = max(10, round(12 * scale))
        blf.size(0, size)
        value = f"{self.height:.0%} · {self.count or 0} Scheite" if self.setups else f"{self.height:.0%} · Holzquelle wählen"
        width = blf.dimensions(0, value)[0] + 20 * scale
        x = knob.x + 18 * scale
        if x + width + 24 * scale > region.width:
            x = knob.x - width - 42 * scale
        y = max(2 * scale, min(knob.y - 12 * scale, region.height - 27 * scale))
        button = (x + width, y, x + width + 24 * scale, y + 24 * scale)
        self.rail = bottom, top, w0, w1, button
        shader = gpu.shader.from_builtin('UNIFORM_COLOR')

        def rect(x1, y1, x2, y2, color):
            self.draw_batch(shader, 'TRI_STRIP', ((x1, y1), (x2, y1), (x1, y2), (x2, y2)), color)

        def circle(center, radius, color):
            points = [center + Vector((math.cos(i * math.tau / 24), math.sin(i * math.tau / 24))) * radius
                      for i in range(25)]
            self.draw_batch(shader, 'TRI_FAN', (center, *points), color)

        blend = gpu.state.blend_get()
        try:
            gpu.state.blend_set('ALPHA')
            rail_color = (0.13, 0.20, 0.22, 0.95)
            rail_normal = (top - bottom).normalized().orthogonal() * 2 * scale
            self.draw_batch(shader, 'TRI_STRIP',
                            (bottom + rail_normal, bottom - rail_normal, top + rail_normal, top - rail_normal),
                            rail_color)
            self.draw_batch(shader, 'TRI_STRIP',
                            (bottom + rail_normal, bottom - rail_normal, knob + rail_normal, knob - rail_normal),
                            self.accent)
            circle(knob, 10 * scale, rail_color)
            circle(knob, 7 * scale, self.accent)
            rect(x, y, x + width, y + 24 * scale, (0.066, 0.09, 0.11, 0.96))
            rect(*button, (0.18, 0.33, 0.33, 1))
            blf.color(0, 0.94, 0.96, 0.98, 1.0)
            blf.position(0, x + 10 * scale, y + 7 * scale, 0)
            blf.draw(0, value)
            bx = button[0]
            self.draw_batch(shader, 'LINE_STRIP', ((bx + 6 * scale, y + 12 * scale),
                            (bx + 10 * scale, y + 8 * scale), (bx + 18 * scale, y + 16 * scale)), self.accent)
        finally:
            gpu.state.blend_set(blend)

@bpy.app.handlers.persistent
def close_height_editors(_unused):
    for editor in tuple(FWSHELF_OT_adjust_height.sessions.values()):
        editor.close()


class FWSHELF_PT_panel(bpy.types.Panel):
    bl_label = "Firewood Shelf Filler"
    bl_idname = "FWSHELF_PT_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Firewood"

    def draw(self, context):
        layout = self.layout
        props = context.scene.fw_shelf_settings

        col = layout.column(align=True)
        col.label(text="1. Quellen & Regal", icon='OUTLINER_COLLECTION')
        col.prop(props, "source_collection", text="Holzscheite")
        col.prop(props, "volume_object", text="Innenraum")
        col.operator("fw_shelf.make_volume", icon='MESH_CUBE', text="Fuellwuerfel erstellen")
        col.label(text="Cube: X=Breite, Y=Tiefe, Z=Hoehe")

        layout.separator()
        col = layout.column(align=True)
        col.label(text="2. Stapel einstellen", icon='MOD_ARRAY')
        col.operator("fw_shelf.small_preset", icon='PHYSICS', text="Kleine Scheite + viele (Preset)")
        col.prop(props, "manual_count")
        row = col.row()
        row.enabled = props.manual_count == 0
        row.prop(props, "fill", slider=True)
        col.prop(props, "diameter")
        row = col.row()
        row.enabled = props.manual_count == 0
        row.prop(props, "max_auto_count")
        col.prop(props, "fill_height", slider=True)
        col.operator("fw_shelf.adjust_height", icon='VIEW3D')
        vol = props.volume_object
        if vol is not None and vol.type == 'MESH':
            try:
                _, dims = fill_geometry(*volume_geometry(vol), props.fill_height)
            except RuntimeError as exc:
                col.label(text=str(exc), icon='ERROR')
            else:
                capacity = slot_grid(dims, props.diameter)[-1]
                col.label(text=f"Startpositionen: {planned_count(props, capacity)}", icon='INFO')
        col.prop(props, "depth_factor", slider=True)
        col.prop(props, "seed")

        box = layout.box()
        box.label(text="Physik (nur beim Erzeugen)", icon='PHYSICS')
        box.prop(props, "frames")
        box.prop(props, "substeps")

        layout.separator()
        col = layout.column(align=True)
        col.scale_y = 1.5
        col.operator("fw_shelf.generate", icon='PLAY', text="3. Holzstapel erzeugen")
        layout.operator("fw_shelf.clear", icon='TRASH', text="Generierten Stapel loeschen")
        layout.label(text="Nach Simulation: statische Objekte")
        layout.label(text="Mesh-Daten geteilt; keine Dauerphysik")


CLASSES = (
    FWShelfSettings,
    FWSHELF_OT_small_preset,
    FWSHELF_OT_make_volume,
    FWSHELF_OT_generate,
    FWSHELF_OT_clear,
    FWSHELF_OT_adjust_height,
    FWSHELF_PT_panel,
)


def register():
    for cls in CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.fw_shelf_settings = PointerProperty(type=FWShelfSettings)
    bpy.app.handlers.load_pre.append(close_height_editors)
    bpy.app.handlers.save_pre.append(close_height_editors)


def unregister():
    close_height_editors(None)
    bpy.app.handlers.load_pre.remove(close_height_editors)
    bpy.app.handlers.save_pre.remove(close_height_editors)
    del bpy.types.Scene.fw_shelf_settings
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
