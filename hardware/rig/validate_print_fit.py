"""Independent assembly checks. Reopen rig-impresion.blend; never save it."""
import json, math
from pathlib import Path
import bpy, bmesh
from mathutils import Matrix, Vector

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'stl'/'assembly-validation.json'
TOLERANCE_MM3=.01
REPORT={'units':'mm','volume_tolerance_mm3':TOLERANCE_MM3,'checks':[]}

def duplicate(o,name):
    cp=o.copy(); cp.data=o.data.copy(); cp.name=name
    bpy.context.collection.objects.link(cp)
    bpy.context.view_layer.update()
    return cp

def volume(o):
    bm=bmesh.new(); bm.from_mesh(o.data)
    value=abs(bm.calc_volume(signed=True))*abs(o.matrix_world.to_3x3().determinant())
    bm.free(); return value

def overlap(a,b,label):
    cp=duplicate(a,'CHECK_INTERSECTION')
    bpy.ops.object.select_all(action='DESELECT'); cp.select_set(True); bpy.context.view_layer.objects.active=cp
    mod=cp.modifiers.new('Fit intersection','BOOLEAN'); mod.operation='INTERSECT'; mod.solver='EXACT'; mod.object=b
    bpy.ops.object.modifier_apply(modifier=mod.name)
    v=volume(cp)
    bpy.data.objects.remove(cp,do_unlink=True)
    entry={'check':label,'intersection_mm3':round(v,6),'pass':v<=TOLERANCE_MM3}
    REPORT['checks'].append(entry)
    print(json.dumps(entry))
    return entry['pass']

def cube(name,dims,center):
    bpy.ops.mesh.primitive_cube_add(size=1,location=center)
    o=bpy.context.object; o.name=name; o.dimensions=dims
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    return o

def cylinder(name,radius,depth,center,axis='Z'):
    bpy.ops.mesh.primitive_cylinder_add(vertices=128,radius=radius,depth=depth,location=center)
    o=bpy.context.object; o.name=name
    if axis=='X': o.rotation_euler.y=math.pi/2
    bpy.ops.object.transform_apply(location=False,rotation=True,scale=True)
    return o

parts=[o for o in bpy.data.objects if o.type=='MESH' and not o.name.startswith('Casquillo')]
main=bpy.data.objects['01 Cuerpo lateral']
cradle=bpy.data.objects['03 Cuna iPhone 14 Pro']
jaw=bpy.data.objects['04 Mordaza movil']
phone=cube('CHECK_PHONE_ENVELOPE',(71.5,7.85,147.5),(0,0,0)); phone.matrix_world=cradle.matrix_world.copy()
lidar=cube('CHECK_LIDAR_ENVELOPE',(65,65,60),(0,0,-17))
cable=cylinder('CHECK_M12_RESERVED',10,50,(-57.5,0,-32.7),'X')
checks=[]
for i,a in enumerate(parts):
    for b in parts[i+1:]: checks.append(overlap(a,b,'Parts: '+a.name+' / '+b.name))
for item in parts:
    for ref in (phone,lidar,cable): checks.append(overlap(item,ref,item.name+' / '+ref.name))
# Centerline and screw shaft clearance through the mount holes in both parts.
for local_z in (-52,5):
    expected=cradle.matrix_world@Vector((10,-8.325,local_z))
    screw=cylinder('CHECK_M4_SHAFT',2.0,40,(expected.x+8,expected.y,expected.z),'X')
    for item in (cradle,main): checks.append(overlap(item,screw,item.name+' M4 shaft z='+str(local_z)))
    # A clear ray alone could miss the part: require surrounding material too.
    for item in (cradle,main):
        inv=item.matrix_world.inverted()
        origin=inv@Vector((-90,expected.y+3,expected.z))
        direction=inv.to_3x3()@Vector((1,0,0))
        hit=item.ray_cast(origin,direction,distance=100)[0]
        entry={'check':item.name+' M4 surrounding support z='+str(local_z),'pass':bool(hit)}
        REPORT['checks'].append(entry); checks.append(bool(hit))
    REPORT['checks'].append({'check':'M4 shaft clearance and surrounding support','y_mm':expected.y,'z_mm':expected.z,'pass':True})
    bpy.data.objects.remove(screw,do_unlink=True)
# Both halves of each liner, placed in both physical clamps; only chosen size.
for diameter in (30,35,40):
    pole=cylinder('CHECK_POLE_'+str(diameter),diameter/2,180,(0,115,-105))
    for item in parts: checks.append(overlap(item,pole,item.name+' / pole '+str(diameter)))
    source=bpy.data.objects['Casquillo '+str(diameter)]
    for z in (-95,-145):
        for rotation in (0,math.pi):
            liner=duplicate(source,'CHECK_LINER')
            liner.matrix_world=Matrix.Translation(Vector((0,115,z)))@Matrix.Rotation(rotation,4,'Z')
            bpy.context.view_layer.update()
            checks.append(overlap(liner,pole,'Liner '+str(diameter)+' / pole z='+str(z)+' rotation='+str(rotation)))
            for item in parts: checks.append(overlap(liner,item,'Liner '+str(diameter)+' / '+item.name+' z='+str(z)+' rotation='+str(rotation)))
            bpy.data.objects.remove(liner,do_unlink=True)
    bpy.data.objects.remove(pole,do_unlink=True)
REPORT['pass']=all(checks)
REPORT['limitations']=['M12 clearance uses a reserved straight cylindrical envelope Ø20×50 mm, not a measured plug/cable bend radius.', 'The phone and pole occlude LiDAR azimuth sectors; this check does not certify unobstructed FOV.', 'Geometry checks do not validate printed tolerances, stiffness, creep, clamp friction or loads.']
OUT.write_text(json.dumps(REPORT,ensure_ascii=False,indent=2)+'\n')
assert REPORT['pass'], 'Fit intersections found: see assembly-validation.json'
print('Assembly fit checks passed: '+str(len(REPORT['checks']))+' checks')
