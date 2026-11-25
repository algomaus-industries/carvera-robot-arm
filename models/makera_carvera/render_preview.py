"""Preview render: blender -b --python render_preview.py -- <out_dir> <png> [lid_angle]"""
import numpy; numpy.bool = bool
import bpy, sys, math, os
a = sys.argv[sys.argv.index("--") + 1:]
out, png = a[0], a[1]; ang = float(a[2]) if len(a) > 2 else 1.4866
bpy.ops.wm.read_factory_settings()
for o in list(bpy.data.objects): bpy.data.objects.remove(o)
def imp(p):
    before = set(bpy.data.objects)
    bpy.ops.import_scene.gltf(filepath=os.path.abspath(p))
    new = [o for o in bpy.data.objects if o not in before]
    import mathutils
    fix = mathutils.Matrix.Rotation(-math.pi / 2, 4, "X")  # GLBs are Z-up; importer assumes Y-up
    for o in new:
        if o.parent is None: o.matrix_world = fix @ o.matrix_world
    return new
imp(f"{out}/makera_carvera/carvera_body.glb")
piv = bpy.data.objects.new("pivot", None); bpy.context.scene.collection.objects.link(piv)
piv.location = (0, 0.100, 0.541); bpy.context.view_layer.update()
for o in imp(f"{out}/makera_carvera/carvera_lid.glb"):
    if o.parent is None:
        mw = o.matrix_world.copy(); o.parent = piv; o.matrix_world = mw
piv.rotation_euler = (-ang, 0, 0)
for y in (0.0305, 0.0005, -0.0295, -0.0595, -0.0895, -0.1195):
    for o in imp(f"{out}/carvera_tool_holder/carvera_tool_holder.glb"):
        if o.parent is None: o.location = (0.205 + o.location[0], y + o.location[1], 0.150 + o.location[2])
cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam")); bpy.context.scene.collection.objects.link(cam)
cam.location = tuple(float(v) for v in os.environ.get("CAM", "1.15,-1.75,1.35").split(","))
d = cam.constraints.new("TRACK_TO"); t = bpy.data.objects.new("t", None); bpy.context.scene.collection.objects.link(t)
t.location = tuple(float(v) for v in os.environ.get("TARGET", "0,0,0.42").split(",")); d.target = t; d.track_axis = "TRACK_NEGATIVE_Z"; d.up_axis = "UP_Y"
bpy.context.scene.camera = cam
s = bpy.context.scene; s.render.engine = "BLENDER_WORKBENCH"
s.display.shading.light = "STUDIO"; s.display.shading.studiolight_rotate_z = 2.4; s.display.shading.show_cavity = True; s.display.shading.color_type = "MATERIAL"
s.render.resolution_x, s.render.resolution_y = 1000, 800; s.render.filepath = os.path.abspath(png)
bpy.ops.render.render(write_still=True)
