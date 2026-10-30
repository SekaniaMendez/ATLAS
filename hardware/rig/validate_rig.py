"""Run inside Blender against the saved blend; changes are not saved."""
import json
import sys
import types
from pathlib import Path
import bpy
from mathutils import Vector

module=types.ModuleType('atlas_rig_validation_tools')
sys.modules[module.__name__]=module
exec(compile(bpy.data.texts['ATLAS_RIG.py'].as_string(),'ATLAS_RIG.py','exec'),module.__dict__)
root=bpy.data.objects['ATLAS_PARAMETERS']
assert root['profile_id']=='iphone14pro-mid360-upright-v0'
assert root['pole_diameter_mm']==35
assert root['pole_offset_y_mm']==115
assert bpy.data.objects['GNSS_RESERVED_FRAME'].location.z==260
pole=bpy.data.objects['Bastón • diámetro editable']
assert abs(pole.location.y-115)<.001
assert pole.location.z+pole.dimensions.z/2 > bpy.data.objects['CAMERA_OPTICAL_FRAME'].location.z
assert bpy.context.scene.unit_settings.scale_length > .00099
assert bpy.context.scene.camera is not None
camera=bpy.data.objects['CAMERA_OPTICAL_FRAME']
assert (camera.matrix_world.translation-Vector((-31,-25,75))).length < .001
output=Path(bpy.data.filepath).resolve().parents[1]/'../build/rig-reference/validation-calibration.json'
output.parent.mkdir(parents=True,exist_ok=True)
module.export_calibration(output)
profile=json.loads(output.read_text())['profiles'][0]
assert not profile['verified']
assert max(abs(a-b) for a,b in zip(profile['lidar_to_camera']['translation_m'],(.075,-.025,-.031))) < 1e-6
# Both extremes rebuild, without empty meshes or losing rendering setup.
for diameter in (30.,40.):
    bpy.data.objects['ATLAS_PARAMETERS']['pole_diameter_mm']=diameter
    assert bpy.ops.atlas.rebuild_rig()=={'FINISHED'}
    assert bpy.context.scene.camera is not None
    assert abs(bpy.data.objects['Bastón • diámetro editable'].dimensions.x-diameter)<.001
    for obj in bpy.data.objects:
        if obj.type=='MESH' and obj.name.startswith(('Casquillo','Abrazadera')):
            assert len(obj.data.vertices)>0 and len(obj.data.polygons)>0, obj.name
# Available profile selection refreshes dimensions and transform.
assert bpy.ops.atlas.select_profile(profile='iphone16promax-mid360-upright-v0')=={'FINISHED'}
assert abs(bpy.data.objects['ATLAS_PARAMETERS']['phone_height_mm']-163.)<.001
assert bpy.ops.atlas.select_profile(profile='iphone14pro-mid360-upright-v0')=={'FINISHED'}
root=bpy.data.objects['ATLAS_PARAMETERS']; root['phone_shift_x_mm']=-25.
assert bpy.ops.atlas.rebuild_rig()=={'FINISHED'}
assert abs(bpy.data.objects['CAMERA_OPTICAL_FRAME'].matrix_world.translation.x+56)<.001
module.export_calibration(output)
shifted=json.loads(output.read_text())['profiles'][0]
assert shifted['lidar_to_camera']['translation_m'] != profile['lidar_to_camera']['translation_m']
assert not shifted['verified']
print('ATLAS rig: reopen, units, optical frame, 30/40 mm rebuild, profile menu and calibration export PASS')
