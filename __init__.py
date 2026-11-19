# -*- coding: utf-8 -*-
bl_info = {
    "name": "Firewood Shelf Filler",
    "author": "OpenAI",
    "version": (1, 3, 6),
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
from bisect import bisect_right
from typing import NamedTuple

import blf
import gpu
from gpu_extras.batch import batch_for_shader
from bpy_extras.view3d_utils import location_3d_to_region_2d
from mathutils import Matrix, Vector, Euler
from mathutils.geometry import convex_hull_2d
from bpy.props import BoolProperty, EnumProperty, FloatProperty, IntProperty, PointerProperty

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
    frame, dims = volume_geometry(volume)
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


def create_walls(temp, frame, dims, meshes, compaction):
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
        body = add_rigidbody(obj, 'PASSIVE', 'BOX')
        if label != "bottom":
            body.friction = FRICTION - 0.62 * compaction
        obj.hide_render = True
        obj.display_type = 'WIRE'


def source_bounds(source):
    points = [vert.co.copy() for vert in source.data.vertices] if source.modifiers else tuple(source.bound_box)
    low = Vector(tuple(min(v[i] for v in points) for i in range(3)))
    high = Vector(tuple(max(v[i] for v in points) for i in range(3)))
    extent = high - low
    scale = source.matrix_world.to_scale()
    axis = max(range(3), key=lambda i: extent[i] * abs(scale[i]))
    return (low + high) * 0.5, extent, scale, axis


def normalize_variant(source, bounds, target_length, target_diameter):
    center, extent, source_scale, axis = bounds
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

def create_proxy_mesh(source, transform, index, seed, guided=False):
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
    if guided:
        section = [Vector((point.x, point.z)) for point in points]
        outline = [section[i] for i in convex_hull_2d(section)]
        if len(outline) >= 3:
            points = [Vector((point.x, y, point.y)) for point in outline for y in (-0.5, 0.5)]
        else:
            # Degenerate sections still need a full-depth collision guide.
            return make_box_mesh(f"__FW_GuideBox_{index:03d}", (0.005, 1.0, 0.005))
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
    row_pitch = pitch * math.sqrt(0.75)
    rows = max(0, math.floor((dims.z - 2 * inset) / row_pitch) + 1)
    capacity = (rows // 2) * (even + odd) + (rows % 2) * even
    return pitch, row_pitch, inset, even, odd, capacity


def planned_count(settings, capacity):
    if settings.manual_count:
        return min(settings.manual_count, capacity)
    return min(capacity, max(1, round(capacity * settings.fill)))


def create_slots(dims, settings):
    pitch, row_pitch, inset, even, odd, capacity = slot_grid(dims, settings.diameter)
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
            z = -dims.z * 0.5 + inset + row * row_pitch
            slots.append((x, z))
        row += 1
    return slots


def prepare_variants(variants, dims, settings):
    rng = random.Random(settings.seed)
    max_length = dims.y - 4 * COLLISION_MARGIN
    setups = []
    for source in variants:
        bounds = source_bounds(source)
        if source.modifiers:
            log(f"{source.name}: Für Instanzen wird das Mesh ohne Modifier verwendet.")
        if settings.length_mode == 'AUTO':
            length = min(dims.y * settings.depth_factor * rng.uniform(*LENGTH_VARIATION),
                         max_length / DIAMETER_VARIATION[1])
            diameter = settings.diameter * rng.uniform(0.93, 1.04)
        else:
            if settings.length_mode == 'FIXED':
                length = settings.log_length
            else:
                _, extent, scale, axis = bounds
                length = extent[axis] * abs(scale[axis])
            diameter = settings.diameter
            if length > max_length:
                units = settings.id_data.unit_settings
                amount = f"{max_length:.3f} Blender-Einheiten"
                if units.system != 'NONE':
                    amount = bpy.utils.units.to_string(units.system, 'LENGTH',
                                                       max_length * units.scale_length, precision=3)
                raise RuntimeError(f"'{source.name}' ist zu lang für das Fach. Maximal passen {amount}.")
        setups.append((source, normalize_variant(source, bounds, length, diameter)))
    return setups


def placements(frame, dims, slots, settings, variant_count):
    rng = random.Random(settings.seed)
    max_length = (dims.y - 4 * COLLISION_MARGIN) / DIAMETER_VARIATION[1]
    for x, z in slots:
        variant = rng.randrange(variant_count)
        angle = Euler((0, rng.uniform(-math.pi, math.pi), 0), 'XYZ')
        scale = 1.0
        y = 0.0
        if settings.length_mode == 'AUTO':
            scale = rng.uniform(*DIAMETER_VARIATION)
            y_room = max(0, min(0.010, (dims.y - max_length * scale) * 0.5 - COLLISION_MARGIN))
            y = rng.uniform(-y_room, y_room)
        p = Vector((x, y, z))
        yield variant, (frame @ Matrix.Translation(p) @ angle.to_matrix().to_4x4()
                        @ Matrix.Scale(scale, 4))


class ProductProfile(NamedTuple):
    variant: int
    rotation: Matrix
    x_min: float
    x_max: float
    z_min: float
    z_max: float
    y_min: float
    y_max: float
    lower: list
    upper: list


def product_profiles(setups, rng):
    profiles = []
    for variant, (source, transform) in enumerate(setups):
        silhouette = []
        y_min, y_max = math.inf, -math.inf
        for vertex in source.data.vertices:
            point = transform @ vertex.co
            silhouette.append(Vector((point.x, point.z)))
            y_min, y_max = min(y_min, point.y), max(y_max, point.y)
        hull = [silhouette[i] for i in convex_hull_2d(silhouette)]
        if len(hull) < 3:
            continue
        area = sum(a.x * b.y - a.y * b.x for a, b in zip(hull, hull[1:] + hull[:1]))
        if area < 0:
            hull.reverse()
        for roll in range(8):
            angle = roll * math.tau / 8 + rng.uniform(-0.12, 0.12)
            rotation = Matrix.Rotation(angle, 4, 'Y')
            cos, sin = math.cos(angle), math.sin(angle)
            corners = [Vector((point.x * cos + point.y * sin, -point.x * sin + point.y * cos))
                       for point in hull]
            lower, upper = [], []
            for a, b in zip(corners, corners[1:] + corners[:1]):
                if a.x + 1e-8 < b.x:
                    lower.append((a.x, b.x, a.y, (b.y - a.y) / (b.x - a.x)))
                elif b.x + 1e-8 < a.x:
                    upper.append((b.x, a.x, b.y, (a.y - b.y) / (a.x - b.x)))
            profiles.append(ProductProfile(
                variant, rotation, min(p.x for p in corners), max(p.x for p in corners),
                min(p.y for p in corners), max(p.y for p in corners), y_min, y_max,
                sorted(lower), sorted(upper),
            ))
    return profiles


def contact_height(lower, upper, offset):
    """Minimum vertical gap between two convex front silhouettes at this X offset."""
    separation = None
    i = j = 0
    while i < len(lower) and j < len(upper):
        ax, ax_end, az, a_slope = lower[i]
        bx, bx_end, bz, b_slope = upper[j]
        bx += offset
        bx_end += offset
        start, end = max(ax, bx), min(ax_end, bx_end)
        if end > start:
            x = end if b_slope > a_slope else start
            diff = bz + (x - bx) * b_slope - az - (x - ax) * a_slope
            separation = diff if separation is None else max(separation, diff)
        if ax_end < bx_end:
            i += 1
        else:
            j += 1
    return separation


def product_layout(dims, settings, setups):
    """Stack supported rows with their long axes parallel to the rack depth."""
    rng = random.Random(settings.seed)
    profiles = product_profiles(setups, rng)
    if not profiles:
        raise RuntimeError("Die Holzquellen brauchen einen dreidimensionalen Querschnitt.")
    narrowest = min(profiles, key=lambda profile: profile.x_max - profile.x_min)
    gap = COLLISION_MARGIN * 0.5
    left, right = -dims.x * 0.5 + gap, dims.x * 0.5 - gap
    floor, ceiling = -dims.z * 0.5, dims.z * 0.5
    result = []
    obstacles = []
    columns = {}
    row = 0
    while True:
        row_start = len(result)
        edge = left + (settings.diameter * 0.38 if row % 2 else 0.0)
        while edge + narrowest.x_max - narrowest.x_min <= right:
            profile = rng.choice(profiles)
            if edge + profile.x_max - profile.x_min > right:
                profile = narrowest
            x = edge - profile.x_min
            end = x + profile.x_max
            bins = range(math.floor(edge / settings.diameter), math.floor(end / settings.diameter) + 1)
            edge = end + gap
            z = floor + gap - profile.z_min
            required_height = floor
            supports = False
            nearby = {index for column in bins for index in columns.get(column, ())}
            for index in nearby:
                old_x, old_z, old_profile, old_height, old_row = obstacles[index]
                contact = contact_height(profile.lower, old_profile.upper, old_x - x)
                if contact is not None:
                    supports |= old_row == row - 1
                    z = max(z, old_z + contact + gap)
                    required_height = max(required_height, old_height)
            if row and not supports:
                continue
            required_height = max(required_height, z + profile.z_max + gap)
            depth_room = dims.y - (profile.y_max - profile.y_min)
            front_margin = min(depth_room * 0.5, gap + rng.uniform(0.002, 0.009))
            y = -dims.y * 0.5 - profile.y_min + front_margin
            matrix = Matrix.Translation((x, y, z)) @ profile.rotation
            if required_height <= ceiling:
                result.append((required_height, profile.variant, matrix))
            for column in bins:
                columns.setdefault(column, []).append(len(obstacles))
            obstacles.append((x, z, profile, required_height, row))
        if len(result) == row_start:
            break
        row += 1
    return sorted(result, key=lambda placement: placement[0])


def create_proxies(temp, frame, dims, setups, proxies, slots, settings):
    log(f"{len(setups)} Varianten, {len(slots)} Startpositionen.")

    bodies = []
    shelf_inverse = frame.inverted()
    for i, (variant, matrix) in enumerate(placements(frame, dims, slots, settings, len(setups))):
        obj = bpy.data.objects.new(f"__FW_log_{i:04d}", proxies[variant])
        temp.objects.link(obj)
        depth_scale = 1.0
        if settings.longitudinal_guide:
            # A full-depth collision hull limits sideways rotation inside the rack.
            # The visible log keeps its original length and roll.
            local = shelf_inverse @ matrix
            local.translation.y = 0.0
            clearance = 2 * COLLISION_MARGIN
            depth_scale = (dims.y - clearance) / local.to_scale().y
            matrix = frame @ local @ Matrix.Diagonal((1, depth_scale, 1, 1))
        obj.matrix_world = matrix
        obj.hide_render = True
        obj.display_type = 'WIRE'
        bodies.append((obj, setups[variant], depth_scale))

    objects = [obj for obj, _, _ in bodies]
    with bpy.context.temp_override(selected_objects=objects, selected_editable_objects=objects):
        status = bpy.ops.rigidbody.objects_add(type='ACTIVE')
    if status != {'FINISHED'}:
        raise RuntimeError("Die Physik-Körper konnten nicht erstellt werden.")
    for obj in objects:
        rb = obj.rigid_body
        if rb is None:
            raise RuntimeError(f"Physik-Körper konnte nicht erstellt werden: {obj.name}")
        rb.collision_shape = 'CONVEX_HULL'
        rb.use_margin = True
        rb.collision_margin = COLLISION_MARGIN
        rb.friction = FRICTION - 0.50 * settings.compaction
        rb.restitution = 0.0
        rb.mass = max(0.05, (settings.diameter / 0.095)**2)
        rb.linear_damping = 0.30
        rb.angular_damping = 0.43
        rb.use_deactivation = True
    return bodies


def simulate_and_snapshot(scene, bodies, settings, progress=None):
    world = scene.rigidbody_world
    if world is None:
        raise RuntimeError("Rigid Body World konnte nicht erzeugt werden.")
    world.substeps_per_frame = settings.substeps
    world.solver_iterations = SIM_SOLVER_ITERATIONS + round(12 * settings.compaction)
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
    for obj, setup, depth_scale in bodies:
        evaluated = obj.evaluated_get(depsgraph)
        visible = evaluated.matrix_world @ Matrix.Diagonal((1, 1 / depth_scale, 1, 1))
        snapshots.append((obj, visible, setup, depth_scale))
    return snapshots


def inside_fill(mesh, matrix, half_dims):
    def inside(point):
        return all(abs(value) <= limit + 0.001 for value, limit in zip(point, half_dims))

    # The bounding box handles most logs cheaply. At the wall, check the actual mesh.
    if not mesh.modifiers and all(inside(matrix @ Vector(corner)) for corner in mesh.bound_box):
        return True
    return all(inside(matrix @ vertex.co) for vertex in mesh.data.vertices)


def copy_log_appearance(source, target):
    for src_slot, dst_slot in zip(source.material_slots, target.material_slots):
        if src_slot.link == 'OBJECT':
            dst_slot.link = 'OBJECT'
            dst_slot.material = src_slot.material
    target.color = source.color


def link_log(collection, source, name, matrix):
    obj = bpy.data.objects.new(name, source.data)
    collection.objects.link(obj)
    try:
        copy_log_appearance(source, obj)
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
        parent_inverse = volume.matrix_world.inverted()
        for i, (matrix, (source, transform)) in enumerate(snapshots):
            obj = link_log(final, source, f"FW_Log_{i:04d}_{source.name}", matrix @ transform)
            obj.parent = volume
            obj.matrix_parent_inverse = parent_inverse
            obj.matrix_basis = matrix @ transform
            obj["firewood_generated"] = True
            obj["firewood_variant"] = source.name
    except Exception:
        remove_stack(scene, final)
        raise
    if previous is not None:
        remove_stack(scene, previous)
        final.name = name
    return final


def fit_depth(matrix, setup, frame, shelf_inverse, depth, *, front_offset=0.0, alignment=0.0):
    source, transform = setup
    local = shelf_inverse @ matrix @ transform
    limit = depth * 0.5 - COLLISION_MARGIN * 0.25
    if not alignment and not source.modifiers and all(
        abs((local @ Vector(corner)).y) <= limit for corner in source.bound_box
    ):
        return matrix, local

    dx, dy, dz, bias = local[1]
    positions = [dx * vert.co.x + dy * vert.co.y + dz * vert.co.z + bias
                 for vert in source.data.vertices]
    minimum_y, maximum_y = min(positions), max(positions)
    lower_shift = -limit - minimum_y
    upper_shift = limit - maximum_y
    if lower_shift > upper_shift:
        return None, local
    front_shift = lower_shift + front_offset
    shift = min(max(alignment * front_shift, lower_shift), upper_shift)
    adjusted = matrix.copy()
    adjusted.translation += frame.to_3x3() @ Vector((0, shift, 0))
    local.translation.y += shift
    return adjusted, local


def settle_stack(scene, frame, dims, slots, settings, setups, proxies, progress):
    accepted = []
    discarded = 0
    shelf_inverse = frame.inverted()
    ceiling = dims.z * 0.5
    top = -ceiling
    batch_frame, batch_dims = frame, dims
    top_up = settings.manual_count == 0 and settings.fill >= 1.0
    rng = random.Random(settings.seed + 3001)
    front_variation = min(settings.diameter * 0.06, 0.004)
    alignment = settings.front_alignment
    if settings.longitudinal_guide:
        # Guided hulls settle centered in depth, including when the visible log is short.
        alignment *= 2 - alignment

    while slots:
        bodies = create_proxies(scene.collection, batch_frame, batch_dims, setups, proxies, slots, settings)
        settled = simulate_and_snapshot(scene, bodies, settings, progress=progress)
        current, rejected = [], []
        new_top = top
        for obj, matrix, setup, depth_scale in settled:
            source, _ = setup
            matrix, local = fit_depth(
                matrix, setup, frame, shelf_inverse, dims.y,
                front_offset=COLLISION_MARGIN * 2 + rng.uniform(0.0, front_variation),
                alignment=alignment,
            )
            if matrix is None or not inside_fill(source, local, dims * 0.5):
                rejected.append(obj)
                continue
            current.append((obj, matrix, setup, depth_scale))
            if top_up:
                # The exact shape matters here. A rotated bounding box can overstate the fill height.
                if source.modifiers or max((local @ Vector(corner)).z for corner in source.bound_box) > new_top:
                    new_top = max(new_top, max((local @ vert.co).z for vert in source.data.vertices))
        accepted.extend((matrix, setup) for _, matrix, setup, _ in current)
        discarded += len(rejected)
        if not top_up or new_top - top < settings.diameter * 0.1:
            break
        top = new_top
        batch_dims = dims.copy()
        batch_dims.z = ceiling - top
        slots = create_slots(batch_dims, settings)
        if not slots:
            break
        batch_frame = frame @ Matrix.Translation((0, 0, (ceiling + top) * 0.5))
        log(f"Freien Bereich von {batch_dims.z:.3f} Blender-Einheiten nachfüllen.")
        scene.frame_set(1)
        for obj in rejected:
            bpy.data.objects.remove(obj, do_unlink=True)
        for obj, matrix, _, depth_scale in current:
            obj.rigid_body.type = 'PASSIVE'
            if settings.longitudinal_guide:
                # Keep the extended hull in the rack's center when it becomes a support.
                local = shelf_inverse @ matrix
                local.translation.y = 0.0
                matrix = frame @ local
            obj.matrix_world = matrix @ Matrix.Diagonal((1, depth_scale, 1, 1))

    return accepted, discarded


def main(context, settings, progress=None):
    scene = context.scene
    context.view_layer.update()
    variants, frame, dims = check_input(scene, settings)
    setups = prepare_variants(variants, dims, settings)
    frame, dims = fill_geometry(frame, dims, settings.fill_height)
    if settings.stack_mode == 'PRODUCT':
        layout = product_layout(dims, settings, setups)
        count = planned_count(settings, len(layout))
        if not count:
            raise RuntimeError("Die Füllhöhe oder Regalbreite reicht für die eingestellte Scheit-Dicke nicht aus.")
        stack = [(frame @ matrix, setups[variant]) for _, variant, matrix in layout[:count]]
        return create_final(scene, settings.volume_object, stack), 0

    slots = create_slots(dims, settings)
    if not slots:
        raise RuntimeError("Die Füllhöhe oder Regalbreite reicht für die eingestellte Scheit-Dicke nicht aus.")
    meshes = []
    proxies = []
    simulation = bpy.data.scenes.new("Firewood Physics")
    try:
        with context.temp_override(scene=simulation, view_layer=simulation.view_layers[0]):
            try:
                simulation.frame_end = settings.frames
                bpy.ops.rigidbody.world_add()
                # The slight pull toward the front lets the logs settle against a common face.
                simulation.gravity += frame.to_3x3() @ Vector((0, -2.75 * settings.front_alignment, 0))
                create_walls(simulation.collection, frame, dims, meshes, settings.compaction)
                for index, (source, transform) in enumerate(setups):
                    mesh = create_proxy_mesh(source, transform, index, settings.seed, settings.longitudinal_guide)
                    meshes.append(mesh)
                    proxies.append(mesh)
                valid, discarded = settle_stack(simulation, frame, dims, slots, settings, setups, proxies, progress)
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

    if not valid:
        raise RuntimeError("Kein Scheit blieb im Füllbereich. Prüfe Füllhöhe, Dicke und Simulationsdauer.")
    return create_final(scene, settings.volume_object, valid), discarded

class FWShelfSettings(bpy.types.PropertyGroup):
    source_collection: PointerProperty(
        name="Holzscheite", type=bpy.types.Collection,
        description="Collection mit den urspruenglichen Holzscheit-Meshes",
    )
    volume_object: PointerProperty(
        name="Regal-Innenraum", type=bpy.types.Object,
        description="Cube, der den freien Platz im Regal ausfuellt (X=Breite, Y=Tiefe, Z=Hoehe)",
    )
    stack_mode: EnumProperty(
        name="Stapelstil",
        items=(
            ('PRODUCT', "Produktbild", "Versetzte Reihen mit parallel ausgerichteten Scheiten"),
            ('PHYSICS', "Physik", "Scheite frei fallen und sich auf natürliche Weise setzen lassen"),
        ),
        default='PRODUCT',
    )
    fill: FloatProperty(
        name="Rasterbelegung", description="Anteil der verfügbaren Positionen innerhalb der gewählten Füllhöhe",
        default=1.0, min=0.10, max=1.0, subtype='FACTOR',
    )
    fill_height: FloatProperty(
        name="Füllhöhe", description="Höhe ab Regalboden. 1,0 entspricht der Regaloberkante",
        default=1.0, min=0.0, subtype='FACTOR',
    )
    manual_count: IntProperty(
        name="Anzahl (0 = Auto)", description="0 = passend zur Regalbreite/-hoehe berechnen",
        default=0, min=0, soft_max=2000,
    )
    diameter: FloatProperty(
        name="Scheit-Dicke", description="Maximaler Querschnittsdurchmesser bei fester Länge oder Quellmaß",
        default=0.055, min=0.015, max=1.0, precision=3, subtype='DISTANCE',
    )
    length_mode: EnumProperty(
        name="Scheitlänge",
        items=(
            ('AUTO', "Nach Regaltiefe", "Leicht variierende Längen als Anteil der Regaltiefe"),
            ('FIXED', "Exakte Länge", "Alle Scheite erhalten dieselbe eingegebene Länge"),
            ('SOURCE', "Länge der Quelle", "Übernimmt die in der Szene gemessenen Quelllängen"),
        ),
        default='AUTO',
    )
    log_length: FloatProperty(
        name="Länge", description="Exakte Länge jedes Holzscheits",
        default=0.25, min=0.001, soft_max=1.0, precision=3, subtype='DISTANCE', unit='LENGTH',
    )
    depth_factor: FloatProperty(
        name="Laenge / Regaltiefe", description="Typische Scheitlaenge relativ zur Regaltiefe",
        default=0.76, min=0.40, max=0.90, subtype='FACTOR',
    )
    seed: IntProperty(
        name="Zufalls-Seed", default=42, min=0, max=999999,
    )
    compaction: FloatProperty(
        name="Verdichtung",
        description="Lässt Scheite leichter aneinander und an den Seitenwänden entlanggleiten",
        default=0.70, min=0.0, max=1.0, subtype='FACTOR',
    )
    longitudinal_guide: BoolProperty(
        name="Längsführung",
        description="Hält die Scheite beim Setzen längs zur Regaltiefe",
        default=True,
    )
    front_alignment: FloatProperty(
        name="Frontausrichtung",
        description="Richtet die vordersten Holzkanten an der Vorderseite des Füllwürfels aus (lokales −Y)",
        default=0.90, min=0.0, max=1.0, subtype='FACTOR',
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
        p.stack_mode = 'PHYSICS'
        p.diameter = 0.055
        p.fill = 1.0
        p.frames = 220
        p.substeps = 14
        self.report({'INFO'}, "Physik-Preset: 5,5 cm, volle Rasterbelegung.")
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


def create_preview_nodes(setups):
    tree = bpy.data.node_groups.new("Firewood Live-Instanzen", 'GeometryNodeTree')
    try:
        tree.interface.new_socket(name="Geometry", in_out='INPUT', socket_type='NodeSocketGeometry')
        tree.interface.new_socket(name="Geometry", in_out='OUTPUT', socket_type='NodeSocketGeometry')
        nodes = tree.nodes
        link = tree.links.new
        inputs = nodes.new('NodeGroupInput')
        output = nodes.new('NodeGroupOutput')
        index = nodes.new('GeometryNodeInputIndex')
        selection = nodes.new('FunctionNodeCompare')
        selection.data_type = 'INT'
        selection.operation = 'LESS_THAN'
        # Older releases also expose sockets for the inactive comparison types.
        operands = {socket.name: socket for socket in selection.inputs if socket.enabled}
        link(index.outputs[0], operands["A"])
        count_socket = operands["B"]
        count_socket.default_value = 0

        instances = nodes.new('GeometryNodeInstanceOnPoints')
        link(inputs.outputs["Geometry"], instances.inputs["Points"])
        link(selection.outputs[0], instances.inputs["Selection"])
        instances.inputs["Pick Instance"].default_value = True
        sources = nodes.new('GeometryNodeGeometryToInstance')
        link(sources.outputs[0], instances.inputs["Instance"])

        # Blender prepends connections to a multi-input socket.
        for source, transform in reversed(setups):
            info = nodes.new('GeometryNodeObjectInfo')
            info.transform_space = 'ORIGINAL'
            info.inputs["Object"].default_value = source
            local = nodes.new('GeometryNodeTransform')
            translation, rotation, scale = transform.decompose()
            local.inputs["Translation"].default_value = translation
            local.inputs["Rotation"].default_value = rotation.to_euler()
            local.inputs["Scale"].default_value = scale
            link(info.outputs["Geometry"], local.inputs["Geometry"])
            link(local.outputs["Geometry"], sources.inputs["Geometry"])

        for input_name, attribute_name, data_type in (
            ("Instance Index", "fw_variant", 'INT'),
            ("Rotation", "fw_rotation", 'FLOAT_VECTOR'),
            ("Scale", "fw_scale", 'FLOAT_VECTOR'),
        ):
            attribute = nodes.new('GeometryNodeInputNamedAttribute')
            attribute.data_type = data_type
            attribute.inputs["Name"].default_value = attribute_name
            link(attribute.outputs["Attribute"], instances.inputs[input_name])
        link(instances.outputs["Instances"], output.inputs["Geometry"])
        return tree, count_socket
    except Exception:
        bpy.data.node_groups.remove(tree)
        raise


def rail_fraction(height, w0, w1):
    if height > 1.0:
        return 1.0 + (height - 1.0) * w0 / w1
    return height * w1 / ((1 - height) * w0 + height * w1)


def rail_height(fraction, w0, w1):
    # Continue past the shelf with the same slope at its top, even in perspective.
    if fraction > 1.0:
        return 1.0 + (fraction - 1.0) * w1 / w0
    return fraction * w0 / ((1 - fraction) * w1 + fraction * w0)


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
        self.volume_inverse = volume.matrix_world.inverted()
        self.initial_frame = self.frame.copy()
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
                return {'CANCELLED'}
        self.preview = None
        self.preview_mesh = None
        self.preview_tree = None
        self.product_heights = None
        self.preview_sources = []
        self.draw_handler = None
        self.previous_layer = None
        self.previous_hidden = False
        self.rail = None
        self.count = None
        self.dragging = False
        self.timer = None
        self.sessions[self.scene_id] = self
        try:
            self.draw_handler = bpy.types.SpaceView3D.draw_handler_add(self.draw_hud, (), 'WINDOW', 'POST_PIXEL')
            if self.setups:
                self.build_preview()
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
        if self.draw_handler is not None:
            bpy.types.SpaceView3D.draw_handler_remove(self.draw_handler, 'WINDOW')
        if self.timer is not None:
            self.wm.event_timer_remove(self.timer)
        if self.preview is not None:
            bpy.data.objects.remove(self.preview, do_unlink=True)
        if self.preview_mesh is not None:
            bpy.data.meshes.remove(self.preview_mesh)
        if self.preview_tree is not None:
            bpy.data.node_groups.remove(self.preview_tree)
        for source in self.preview_sources:
            bpy.data.objects.remove(source)
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

    def populate_preview_points(self, height):
        frame, dims = fill_geometry(Matrix.Identity(4), self.dims, height)
        positions, variants, rotations, scales = [], [], [], []
        if self.settings.stack_mode == 'PRODUCT':
            layout = product_layout(dims, self.settings, self.setups)
            self.product_heights = [threshold + frame.translation.z for threshold, _, _ in layout]
            poses = ((variant, frame @ matrix) for _, variant, matrix in layout)
        else:
            slots = create_slots(dims, self.settings)
            poses = placements(frame, dims, slots, self.settings, len(self.setups))
        for variant, matrix in poses:
            translation, rotation, scale = matrix.decompose()
            positions.append(translation)
            variants.append(variant)
            rotations.extend(rotation.to_euler())
            scales.extend(scale)

        self.preview_mesh.clear_geometry()
        self.preview_mesh.from_pydata(positions, [], [])
        for name, data_type, field, values in (
            ("fw_variant", 'INT', "value", variants),
            ("fw_rotation", 'FLOAT_VECTOR', "vector", rotations),
            ("fw_scale", 'FLOAT_VECTOR', "vector", scales),
        ):
            attribute = self.preview_mesh.attributes.get(name)
            if attribute is None:
                attribute = self.preview_mesh.attributes.new(name, data_type, 'POINT')
            attribute.data.foreach_set(field, values)
        self.preview_capacity_height = height

    def build_preview(self):
        self.preview_mesh = bpy.data.meshes.new("Firewood Live-Punkte")
        self.populate_preview_points(max(1.0, self.height))
        node_setups = []
        for source, transform in self.setups:
            if source.modifiers:
                raw_source = bpy.data.objects.new("Firewood Vorschauquelle", source.data)
                self.preview_sources.append(raw_source)
                copy_log_appearance(source, raw_source)
                source = raw_source
            node_setups.append((source, transform))
        self.preview_tree, self.count_socket = create_preview_nodes(node_setups)
        self.preview = bpy.data.objects.new("Firewood Live-Vorschau", self.preview_mesh)
        self.scene.collection.objects.link(self.preview)
        self.preview.hide_select = True
        self.preview.hide_render = True
        self.preview.parent = self.volume
        self.preview.matrix_parent_inverse = self.volume.matrix_world.inverted()
        self.preview.matrix_basis = self.frame
        modifier = self.preview.modifiers.new("Scheite nach Füllhöhe", 'NODES')
        modifier.node_group = self.preview_tree

    def update_preview(self):
        if self.preview is not None and self.height > self.preview_capacity_height:
            self.populate_preview_points(max(self.height, self.preview_capacity_height * 1.5))
            self.preview.update_tag()
        if self.product_heights is not None:
            capacity = bisect_right(self.product_heights, self.dims.z * (self.height - 0.5))
        else:
            _, dims = fill_geometry(self.frame, self.dims, self.height)
            capacity = slot_grid(dims, self.settings.diameter)[-1]
        count = planned_count(self.settings, capacity)
        if count == self.count:
            return
        self.count = count
        if self.preview is not None:
            self.count_socket.default_value = count
            self.preview.update_tag()

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
        if (region is None or context.scene is None or context.scene.as_pointer() != self.scene_id
                or context.scene.fw_shelf_settings.volume_object != self.volume):
            self.close()
            return {'CANCELLED'}

        if event.type == 'TIMER':
            matrix = self.volume.matrix_world @ self.volume_inverse @ self.initial_frame
            if matrix != self.frame:
                self.frame = matrix
                area.tag_redraw()
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
        t = max(0.0, (mouse - bottom).dot(direction) / direction.length_squared)
        if event.type == 'LEFTMOUSE' and event.value == 'PRESS':
            if button[0] <= mouse.x <= button[2] and button[1] <= mouse.y <= button[3]:
                self.update_preview()
                return self.apply_height()
            rail_end = max(1.0, rail_fraction(self.height, w0, w1))
            if (mouse - bottom - min(t, rail_end) * direction).length <= 13 * self.ui_scale:
                self.dragging = True
        if self.dragging and event.type in {'MOUSEMOVE', 'LEFTMOUSE'}:
            snap = 4 * self.ui_scale / direction.length
            if t < snap:
                t = 0.0
            self.height = rail_height(t, w0, w1)
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
        if any(point is None or not all(math.isfinite(value) and abs(value) < 100000
                                          for value in point) for point in (bottom, top)):
            return None
        if (top - bottom).length < 36 * self.ui_scale:
            # A top view foreshortens the shelf's Z axis.
            center = (top + bottom) * 0.5
            offset = Vector((0, 45 * self.ui_scale))
            return center - offset, center + offset, 1.0, 1.0
        w0 = (view.perspective_matrix @ Vector((*start, 1.0))).w
        w1 = (view.perspective_matrix @ Vector((*end, 1.0))).w
        return bottom, top, w0, w1

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
        t = rail_fraction(self.height, w0, w1)
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
        vertices, colors = [], []

        def triangle(a, b, c, color):
            vertices.extend((a, b, c))
            colors.extend((color, color, color))

        def quad(a, b, c, d, color):
            triangle(a, b, c, color)
            triangle(c, b, d, color)

        def rect(x1, y1, x2, y2, color):
            quad((x1, y1), (x2, y1), (x1, y2), (x2, y2), color)

        def line(a, b, width, color):
            normal = (b - a).normalized().orthogonal() * width * 0.5
            quad(a + normal, a - normal, b + normal, b - normal, color)

        def circle(center, radius, color):
            points = [center + Vector((math.cos(i * math.tau / 20), math.sin(i * math.tau / 20))) * radius
                      for i in range(20)]
            for i in range(20):
                triangle(center, points[i - 1], points[i], color)

        half_x, half_y, half_z = self.dims * 0.5
        top_z = -half_z + self.dims.z * self.height
        corners = [location_3d_to_region_2d(region, view, self.frame @ Vector((i * half_x, j * half_y, top_z)))
                   for i, j in ((-1, -1), (1, -1), (-1, 1), (1, 1))]
        if all(point is not None and max(abs(point.x), abs(point.y)) < 100000 for point in corners):
            quad(*corners, (*self.accent[:3], 0.19))
            for i, j in ((0, 1), (1, 3), (3, 2), (2, 0)):
                line(corners[i], corners[j], scale, (*self.accent[:3], 0.65))

        rail_color = (0.13, 0.20, 0.22, 0.95)
        line(bottom, top, 4 * scale, rail_color)
        if knob != bottom:
            line(bottom, knob, 4 * scale, self.accent)
        circle(knob, 10 * scale, rail_color)
        circle(knob, 7 * scale, self.accent)
        rect(x, y, x + width, y + 24 * scale, (0.066, 0.09, 0.11, 0.96))
        rect(*button, (0.18, 0.33, 0.33, 1))
        bx = button[0]
        check = [Vector((bx + x * scale, y + dy * scale)) for x, dy in ((6, 12), (10, 8), (18, 16))]
        line(check[0], check[1], 1.5 * scale, self.accent)
        line(check[1], check[2], 1.5 * scale, self.accent)

        shader = gpu.shader.from_builtin('SMOOTH_COLOR')
        batch = batch_for_shader(shader, 'TRIS', {"pos": vertices, "color": colors})
        blend = gpu.state.blend_get()
        try:
            gpu.state.blend_set('ALPHA')
            batch.draw(shader)
            blf.color(0, 0.94, 0.96, 0.98, 1.0)
            blf.position(0, x + 10 * scale, y + 7 * scale, 0)
            blf.draw(0, value)
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
        col.prop(props, "stack_mode")
        if props.stack_mode == 'PHYSICS':
            col.operator("fw_shelf.small_preset", icon='PHYSICS', text="Kleine Scheite + viele (Preset)")
        col.prop(props, "manual_count")
        row = col.row()
        row.enabled = props.manual_count == 0
        row.prop(props, "fill", slider=True)
        col.prop(props, "diameter")
        col.prop(props, "fill_height")
        col.operator("fw_shelf.adjust_height", icon='VIEW3D')
        vol = props.volume_object
        if props.stack_mode == 'PHYSICS' and vol is not None and vol.type == 'MESH':
            try:
                _, dims = fill_geometry(*volume_geometry(vol), props.fill_height)
            except RuntimeError as exc:
                col.label(text=str(exc), icon='ERROR')
            else:
                capacity = slot_grid(dims, props.diameter)[-1]
                col.label(text=f"Startpositionen: {planned_count(props, capacity)}", icon='INFO')
        col.prop(props, "length_mode")
        if props.length_mode == 'AUTO':
            col.prop(props, "depth_factor", slider=True)
        elif props.length_mode == 'FIXED':
            col.prop(props, "log_length")
        col.prop(props, "seed")

        if props.stack_mode == 'PHYSICS':
            box = layout.box()
            box.label(text="Physik (nur beim Erzeugen)", icon='PHYSICS')
            box.prop(props, "longitudinal_guide")
            box.prop(props, "compaction", slider=True)
            box.prop(props, "front_alignment", slider=True)
            box.prop(props, "frames")
            box.prop(props, "substeps")

        layout.separator()
        col = layout.column(align=True)
        col.scale_y = 1.5
        col.operator("fw_shelf.generate", icon='PLAY', text="3. Holzstapel erzeugen")
        layout.operator("fw_shelf.clear", icon='TRASH', text="Generierten Stapel loeschen")
        layout.label(text="Ergebnis: statische Objekte")
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
    bpy.app.handlers.undo_pre.append(close_height_editors)
    bpy.app.handlers.redo_pre.append(close_height_editors)


def unregister():
    close_height_editors(None)
    bpy.app.handlers.load_pre.remove(close_height_editors)
    bpy.app.handlers.save_pre.remove(close_height_editors)
    bpy.app.handlers.undo_pre.remove(close_height_editors)
    bpy.app.handlers.redo_pre.remove(close_height_editors)
    del bpy.types.Scene.fw_shelf_settings
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
