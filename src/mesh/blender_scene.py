"""dem2mesh's Blender scene (spec 7.2), and Blender's check of the printable solid.

dem2mesh carries this script inside it, writes it to a temporary file and runs

    blender --background --factory-startup --python <file> -- --ply=PATH --stl=PATH --blend=PATH --name=NAME
            --width=W --height=H --zmin=LOW --zmax=HIGH --scale=S

where W and H are the window's size and LOW and HIGH the surface's lowest and highest heights, all in model units, and
S is --scale, model units per metre. The script builds a scene around the terrain surface in the PLY and saves it as
the .blend, then imports the STL into an empty scene and checks that it is a closed solid facing out: a second opinion
on dem2mesh's own check (spec 6), from another program's reading of the file. Each step prints one line starting
"blender: ", which dem2mesh shows with --verbose, and the check prints the line starting "stl check: ", which dem2mesh
always shows.

The exit code is 0 only when every step ran and the check found nothing wrong. Blender itself exits 0 after an
exception in a --python script, having printed it, so every exception is caught at the bottom and made exit 1.

The text is embedded in a C++ raw string (src/mesh/BlenderScenePy.h.in), so it must never hold that string's closing
delimiter; CMake refuses to configure when it does.
"""

import argparse
import math
import sys
import traceback
from array import array

import bmesh
import bpy
from mathutils import Matrix, Vector

# The oldest Blender this is written for (spec 7.1): 4.2 LTS, the first with EEVEE Next.
OLDEST_BLENDER = (4, 2, 0)

RESOLUTION = (1920, 1080)
SAMPLES = 64
LENS_MM = 50.0
MARGIN = 1.05  # the frame holds the terrain with 5 percent to spare
CAMERA_AZIMUTH_DEG = 135.0  # south-east, clockwise from north
CAMERA_ELEVATION_DEG = 35.0
SUN_AZIMUTH_DEG = 315.0  # north-west
SUN_ELEVATION_DEG = 40.0
SUN_STRENGTH = 3.0
TERRAIN_COLOUR = (0.55, 0.50, 0.45, 1.0)  # a warm grey
TERRAIN_ROUGHNESS = 0.9
WORLD_COLOUR = (0.8, 0.8, 0.8)  # the background the camera sees
WORLD_LIGHT = (0.25, 0.25, 0.25)  # the light the world casts on the terrain
PRINT_SIZED = 0.1  # a --scale below this is a model to print, in millimetres


def say(text):
    """One line of the log. Flushed at once: into a pipe Python holds its output back, so Blender's own lines, written
    from C, would overtake it, and a crash would lose it."""
    print("blender: " + text, flush=True)


def parse_arguments():
    """The arguments after "--", which Blender leaves to the script. A usage error exits 2 with argparse's message."""
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser(prog="blender_scene.py", description="dem2mesh's Blender scene and STL check")
    for name in ("ply", "stl", "blend", "name"):
        parser.add_argument("--" + name, required=True)
    for name in ("width", "height", "zmin", "zmax", "scale"):
        parser.add_argument("--" + name, type=float, required=True)
    args = parser.parse_args(argv)
    if not all(math.isfinite(n) for n in (args.width, args.height, args.zmin, args.zmax, args.scale)):
        parser.error("every number must be finite")
    if not (args.width > 0 and args.height > 0 and args.scale > 0 and args.zmax >= args.zmin):
        parser.error("the window and the scale must be above 0, and zmax at least zmin")
    return args


def direction(azimuth_deg, elevation_deg):
    """The unit vector towards an azimuth, in degrees clockwise from north, at an elevation in degrees above the
    horizon: x east, y north, z up, as in the model."""
    azimuth = math.radians(azimuth_deg)
    elevation = math.radians(elevation_deg)
    return Vector((math.cos(elevation) * math.sin(azimuth), math.cos(elevation) * math.cos(azimuth),
                   math.sin(elevation)))


def facing(forward):
    """The rotation that turns a camera or a light, which look down their local -z with +y up, to look along forward
    with the horizon level."""
    return forward.to_track_quat('-Z', 'Y')


def import_terrain(args):
    """Step 1: the surface from the PLY, the object named after the output stem and shaded smooth. The terrain stays
    where the PLY and the STL put it, the window's south-west corner at the origin at sea level (spec 5.2), so that the
    STL imported into the scene lies exactly on it. Its origin, which Blender's tools turn and scale about, is the
    centre of its base, the lowest height under the middle of the window: the object is placed there, and the mesh's
    own coordinates are made relative to it, as Blender's Set Origin does, which leaves every vertex where it was."""
    bpy.ops.wm.ply_import(filepath=args.ply, forward_axis='Y', up_axis='Z')
    objects = list(bpy.context.scene.objects)
    if len(objects) != 1 or objects[0].type != 'MESH' or len(objects[0].data.vertices) == 0:
        raise RuntimeError("importing %s gave %d objects, not one mesh" % (args.ply, len(objects)))
    terrain = objects[0]
    terrain.name = args.name
    terrain.data.name = args.name
    terrain.data.shade_smooth()
    base_centre = Vector((args.width / 2, args.height / 2, args.zmin))
    terrain.data.transform(Matrix.Translation(-base_centre))
    terrain.location = base_centre
    bpy.context.view_layer.update()  # so that matrix_world has the new location
    corners = [terrain.matrix_world @ Vector(corner) for corner in terrain.bound_box]
    low = Vector(tuple(min(c[i] for c in corners) for i in range(3)))
    high = Vector(tuple(max(c[i] for c in corners) for i in range(3)))
    uv = terrain.data.uv_layers.active
    say("imported %s as %s: %d vertices, %d faces, UV map %s, shaded smooth, in place at x %.6g to %.6g, y %.6g to "
        "%.6g, z %.6g to %.6g, origin at the centre of its base"
        % (args.ply, terrain.name, len(terrain.data.vertices), len(terrain.data.polygons),
           uv.name if uv else "missing", low.x, high.x, low.y, high.y, low.z, high.z))
    return terrain


def add_material(terrain):
    """Step 2 (spec 7.2 and 7.3): one matte warm grey, so that the relief reads under the sun, and an image texture node
    that nothing is wired to yet. The node reads the mesh's UV map, made from the PLY's s and t, so draping an aerial
    photo later is loading the photo into it and connecting its colour to the shader."""
    material = bpy.data.materials.new("Terrain")
    material.use_nodes = True
    nodes = material.node_tree.nodes
    shader = next(node for node in nodes if node.type == 'BSDF_PRINCIPLED')
    shader.inputs["Base Color"].default_value = TERRAIN_COLOUR
    shader.inputs["Roughness"].default_value = TERRAIN_ROUGHNESS
    photo = nodes.new("ShaderNodeTexImage")
    photo.name = "AerialPhoto"
    photo.label = "Aerial photo"
    photo.location = (shader.location.x - 400.0, shader.location.y)
    # What the viewport shows in solid mode.
    material.diffuse_color = TERRAIN_COLOUR
    material.roughness = TERRAIN_ROUGHNESS
    terrain.data.materials.append(material)
    say("material Terrain: warm grey, roughness 0.9, image texture AerialPhoto wired to nothing")


def add_sun(scene, centre, reach):
    """Step 3: a sun of strength 3 from the north-west, 40 degrees up: light from the top left of a map, which most
    eyes read as relief. A sun lights the scene the same from anywhere; it is put out along its rays only so that the
    viewport shows it beside the terrain."""
    light = bpy.data.lights.new("Sun", type='SUN')
    light.energy = SUN_STRENGTH
    sun = bpy.data.objects.new("Sun", light)
    towards_sun = direction(SUN_AZIMUTH_DEG, SUN_ELEVATION_DEG)
    sun.rotation_euler = facing(-towards_sun).to_euler()
    sun.location = centre + towards_sun * reach
    scene.collection.objects.link(sun)
    say("sun: strength 3, from the north-west at 40 degrees")


def add_camera(scene, args, centre):
    """Step 4: a 50 mm camera south-east of the terrain, 35 degrees above its centre and looking at it, as near as it
    can be with all eight corners of the terrain's box inside the frame with 5 percent to spare. Fitting the window's
    diagonal across the frame's width alone would not do: seen from 35 degrees up, the nearest corner would drop out of
    the bottom of a 16:9 frame. The viewports are turned to it, so that the file opens on the terrain and not on a grey
    blob or an empty view."""
    data = bpy.data.cameras.new("Camera")
    data.lens = LENS_MM
    camera = bpy.data.objects.new("Camera", data)
    towards_camera = direction(CAMERA_AZIMUTH_DEG, CAMERA_ELEVATION_DEG)
    forward = -towards_camera
    rotation = facing(forward)
    right = rotation @ Vector((1.0, 0.0, 0.0))
    up = rotation @ Vector((0.0, 1.0, 0.0))
    # angle_x spans the frame's width (the sensor's 36 mm lie across a landscape frame); the height is its share of it.
    tan_x = math.tan(data.angle_x / 2)
    tan_y = tan_x * RESOLUTION[1] / RESOLUTION[0]
    corners = [Vector((x, y, z)) - centre
               for x in (0.0, args.width)
               for y in (0.0, args.height)
               for z in (args.zmin, args.zmax)]
    # A corner c from the centre lies c . forward beyond it, so from distance d it is d + c . forward deep, and it is in
    # the frame while MARGIN |c . right| is within tan_x of that depth, and MARGIN |c . up| within tan_y. Never inside
    # the sphere round the box either, so that the camera stays out of it whatever the box's shape.
    distance = max(max(MARGIN * abs(c.dot(right)) / tan_x, MARGIN * abs(c.dot(up)) / tan_y) - c.dot(forward)
                   for c in corners)
    distance = max(distance, MARGIN * max(c.length for c in corners))
    camera.location = centre + towards_camera * distance
    camera.rotation_euler = rotation.to_euler()
    # The terrain lies between about half and twice the distance away; a ratio of a thousand keeps depth precise.
    data.clip_start = distance / 100
    data.clip_end = distance * 10
    scene.collection.objects.link(camera)
    scene.camera = camera
    for screen in bpy.data.screens:
        for area in screen.areas:
            for space in area.spaces:
                if space.type != 'VIEW_3D' or space.region_3d is None:
                    continue
                # Nearer than the camera's, so that zooming in on the surface does not cut into it.
                space.clip_start = distance / 10000
                space.clip_end = data.clip_end
                # The view orbiting returns to is the camera's own.
                space.region_3d.view_location = centre
                space.region_3d.view_rotation = rotation
                space.region_3d.view_distance = distance
                space.region_3d.view_perspective = 'CAMERA'
    say("camera: 50 mm, from the south-east at 35 degrees, %.6g units from the centre; the viewports look through it"
        % distance)


def set_world(scene):
    """Step 5: a light grey background of strength 1 behind the terrain, which lights it more dimly. Lit by the world
    at 0.8 the shadows would be nearly as bright as the sunlit slopes and the relief would wash out, so a Light Path
    node's Is Camera Ray mixes the colour: 0.8 for what the camera sees, 0.25 for the light the world casts."""
    world = bpy.data.worlds.new("World")
    world.use_nodes = True
    tree = world.node_tree
    background = next(node for node in tree.nodes if node.type == 'BACKGROUND')
    background.inputs["Strength"].default_value = 1.0
    path = tree.nodes.new("ShaderNodeLightPath")
    path.location = (background.location.x - 400.0, background.location.y)
    mix = tree.nodes.new("ShaderNodeMix")
    mix.data_type = 'RGBA'
    mix.location = (background.location.x - 200.0, background.location.y)
    # The Mix node has a socket of each name for every data type; the colour ones are told apart by identifier.
    inputs = {socket.identifier: socket for socket in mix.inputs}
    inputs["A_Color"].default_value = WORLD_LIGHT + (1.0,)
    inputs["B_Color"].default_value = WORLD_COLOUR + (1.0,)
    tree.links.new(path.outputs["Is Camera Ray"], inputs["Factor_Float"])
    result = next(socket for socket in mix.outputs if socket.identifier == "Result_Color")
    tree.links.new(result, background.inputs["Color"])
    world.color = WORLD_COLOUR  # the viewport's
    scene.world = world
    say("world: light grey, 0.8, behind the terrain, lighting it at 0.25, strength 1")


def set_render(scene):
    """Step 6: EEVEE, under the identifier this Blender gives it (4.2 to 4.5 call it BLENDER_EEVEE_NEXT), 1920 x 1080,
    64 samples. Cycles is one click away."""
    engines = {item.identifier for item in bpy.types.RenderSettings.bl_rna.properties['engine'].enum_items}
    scene.render.engine = 'BLENDER_EEVEE_NEXT' if 'BLENDER_EEVEE_NEXT' in engines else 'BLENDER_EEVEE'
    scene.render.resolution_x, scene.render.resolution_y = RESOLUTION
    scene.render.resolution_percentage = 100
    scene.eevee.taa_render_samples = SAMPLES
    say("render: %s, %d x %d, %d samples" % (scene.render.engine, RESOLUTION[0], RESOLUTION[1], SAMPLES))


def set_units(scene, scale):
    """Step 7: metric. A print-sized model, --scale under a tenth, shows a unit as a millimetre, the size the slicer
    prints it at. Otherwise a unit shows as the 1 / --scale metres of terrain it stands for, which is a metre at full
    size. The line gives the settings as the scene holds them."""
    units = scene.unit_settings
    units.system = 'METRIC'
    if scale < PRINT_SIZED:
        units.scale_length = 0.001
        units.length_unit = 'MILLIMETERS'
    else:
        units.scale_length = 1.0 / scale
        units.length_unit = 'METERS'
    say("units: %s, scale_length %.6g, length_unit %s" % (units.system, units.scale_length, units.length_unit))


def save(path):
    """Step 8. Blender would keep the file it saves over as a .blend1 beside it, which nobody asked for."""
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=path, compress=True)
    say("saved " + path)


def declared_triangles(path):
    """The number of triangles a binary STL says it holds: bytes 80 to 83, little-endian."""
    with open(path, "rb") as stl:
        stl.seek(80)
        count = stl.read(4)
    if len(count) != 4:
        raise RuntimeError("%s is too short for a binary STL" % path)
    return int.from_bytes(count, "little")


def check_stl(path):
    """Step 9, the second opinion on the STL (spec 6). Blender's own reader imports it into an empty scene, merging the
    corners that the triangles share. That reader drops a triangle that repeats another or has two corners at one
    point, saying so only in a warning, so the count it imports is held against the count the file declares. Its
    validation is off, so that what mesh.validate() finds to mend is seen here too. Then:
    - non-manifold edges are those not shared by exactly two faces: a hole, or a fin of three faces or more;
    - inverted faces are those that Blender's recalculation of outward normals turns round. Each face's normal is kept
      beforehand, 12 bytes a face, and the recalculation is done in place: a copy of the mesh would double the memory,
      several gigabytes on a large grid.
    Prints the "stl check:" line, after the mismatch's own line when the counts differ, and returns whether the solid
    passed."""
    declared = declared_triangles(path)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.wm.stl_import(filepath=path, forward_axis='Y', up_axis='Z', use_mesh_validate=False)
    objects = list(bpy.context.scene.objects)
    if len(objects) != 1 or objects[0].type != 'MESH':
        raise RuntimeError("importing %s gave %d objects, not one mesh" % (path, len(objects)))
    mesh = objects[0].data
    mended = mesh.validate(verbose=True)
    normals = array('f', bytes(12 * len(mesh.polygons)))
    mesh.polygons.foreach_get("normal", normals)
    solid = bmesh.new()
    solid.from_mesh(mesh)  # the faces in the mesh's order, which the normals are in
    triangles = len(solid.faces)
    non_manifold = sum(1 for edge in solid.edges if len(edge.link_faces) != 2)
    bmesh.ops.recalc_face_normals(solid, faces=solid.faces)
    solid.normal_update()
    inverted = sum(1 for i, face in enumerate(solid.faces) if face.normal.dot(normals[3 * i:3 * i + 3]) < 0)
    solid.free()
    say("imported %s into an empty scene; mesh.validate() %s"
        % (path, "mended invalid geometry" if mended else "found nothing to mend"))
    if declared != triangles:
        print("stl check: file declares %d triangles, Blender imported %d" % (declared, triangles), flush=True)
    print("stl check: %d triangles, %d non-manifold edges, %d inverted faces" % (triangles, non_manifold, inverted),
          flush=True)
    return declared == triangles and non_manifold == 0 and inverted == 0 and not mended


def main():
    if bpy.app.version < OLDEST_BLENDER:
        say("this is Blender %s; dem2mesh's scene needs Blender 4.2 or later" % bpy.app.version_string)
        return 1
    args = parse_arguments()
    # An empty scene to build in, not --factory-startup's cube, light and camera. The screens stay.
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    # The middle of the terrain's box, which the camera looks at.
    centre = Vector((args.width / 2, args.height / 2, (args.zmin + args.zmax) / 2))
    terrain = import_terrain(args)
    add_material(terrain)
    add_sun(scene, centre, max(args.width, args.height))
    add_camera(scene, args, centre)
    set_world(scene)
    set_render(scene)
    set_units(scene, args.scale)
    save(args.blend)
    return 0 if check_stl(args.stl) else 1


if __name__ == "__main__":
    try:
        status = main()
    except SystemExit:
        raise  # argparse's usage errors, which carry their own message and code
    except BaseException:
        traceback.print_exc()
        status = 1
    if status:
        sys.stderr.flush()
        sys.exit(status)
