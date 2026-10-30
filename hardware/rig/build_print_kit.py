"""Blender: functional prototype STL kit, coordinates already in millimetres."""
import json, math, struct
from pathlib import Path
import bpy, bmesh
from mathutils import Matrix, Vector
HERE=Path(__file__).resolve().parent
OUT=HERE/'stl'
PARTS=[]; MANIFEST=[]

def select(o):
    bpy.ops.object.select_all(action='DESELECT'); o.select_set(True); bpy.context.view_layer.objects.active=o

def box(name,xyz,dims):
    bpy.ops.mesh.primitive_cube_add(size=1,location=xyz)
    o=bpy.context.object; o.name=name; o.dimensions=dims
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    return o

def cyl(name,xyz,r,depth,axis='Z',vertices=96):
    bpy.ops.mesh.primitive_cylinder_add(vertices=vertices,radius=r,depth=depth,location=xyz)
    o=bpy.context.object; o.name=name
    if axis=='X': o.rotation_euler.y=math.pi/2
    if axis=='Y': o.rotation_euler.x=math.pi/2
    bpy.ops.object.transform_apply(location=False,rotation=True,scale=True)
    return o

def boolean(a,b,op):
    bpy.context.view_layer.update(); select(a)
    m=a.modifiers.new(op,'BOOLEAN'); m.operation=op; m.solver='MANIFOLD'; m.object=b
    bpy.ops.object.modifier_apply(modifier=m.name); bpy.data.objects.remove(b,do_unlink=True)
    return a

def join(a,b): return boolean(a,b,'UNION')
def cut(a,b): return boolean(a,b,'DIFFERENCE')

def beam(name,start,end,width,depth):
    d=Vector(end)-Vector(start)
    o=box(name,(Vector(start)+Vector(end))/2,(d.length,width,depth))
    o.rotation_euler.z=math.atan2(d.y,d.x)
    bpy.ops.object.transform_apply(location=False,rotation=True,scale=True)
    return o

def slot(name,x1,x2,y,z,r,depth):
    # Single watertight extruded capsule; no coplanar unions at the end caps.
    profile=[]
    for cx,begin in ((x1,90),(x2,270)):
        for i in range(49):
            angle=math.radians(begin+180*i/48)
            profile.append((cx+r*math.cos(angle),z+r*math.sin(angle)))
    n=len(profile)
    verts=[(x,y+side*depth/2,zz) for side in (-1,1) for x,zz in profile]
    faces=[tuple(range(n)),tuple(reversed(range(n,2*n)))]
    faces += [(i,n+i,n+(i+1)%n,(i+1)%n) for i in range(n)]
    mesh=bpy.data.meshes.new(name); mesh.from_pydata(verts,[],faces); mesh.update()
    o=bpy.data.objects.new(name,mesh); bpy.context.collection.objects.link(o)
    return o

def bake(o):
    select(o); bpy.ops.object.transform_apply(location=True,rotation=True,scale=True)
    bm=bmesh.new(); bm.from_mesh(o.data)
    bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=1e-4)
    bmesh.ops.dissolve_degenerate(bm,dist=1e-5,edges=list(bm.edges))
    # Preserve planar face partitions around bores: dissolving them can create
    # an annular/self-touching ngon whose triangulation is nonmanifold.
    bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces))
    bmesh.ops.triangulate(bm,faces=list(bm.faces))
    if bm.calc_volume(signed=True)<0: bmesh.ops.reverse_faces(bm,faces=list(bm.faces))
    bad=[e for e in bm.edges if not e.is_manifold]
    if bad: raise RuntimeError(f'{o.name}: {len(bad)} nonmanifold edges')
    bm.to_mesh(o.data); bm.free(); o.data.update()
    return o

def record(o,filename,count,material,rotation=None):
    bake(o); PARTS.append(o); rot=rotation or Matrix.Identity(3)
    coords=[rot@(o.matrix_world@v.co) for v in o.data.vertices]
    lo=Vector(tuple(min(v[a] for v in coords) for a in range(3)))
    coords=[v-lo for v in coords]; o.data.calc_loop_triangles()
    path=OUT/filename; path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('wb') as f:
        f.write(b'ATLAS physical rig prototype; millimetres; no sensors or pole'.ljust(80,b'\0'))
        f.write(struct.pack('<I',len(o.data.loop_triangles)))
        for tri in o.data.loop_triangles:
            a,b,c=(coords[i] for i in tri.vertices); n=(b-a).cross(c-a).normalized()
            f.write(struct.pack('<12fH',*n,*a,*b,*c,0))
    dims=[round(max(v[a] for v in coords),4) for a in range(3)]
    MANIFEST.append({'file':filename,'quantity':count,'suggested_material':material,'dimensions_mm':dims})
    o['stl']=filename; o['quantity']=count
    return o

BORE=21.4; OUTER=28.4; GAP=1.5; EARY=OUTER+5

def clamp(name,sign,center,trap=False):
    x,y,z=center
    o=cyl(name,center,OUTER,24)
    cut(o,cyl('bore',center,BORE,30))
    cut(o,box('split',(x-sign*(OUTER+GAP)/2,y,z),(OUTER+3*GAP,100,30)))
    for side in (-1,1):
        join(o,box('ear',(x+sign*7,y+side*EARY,z),(11,16,24)))
        cut(o,cyl('M4 through',(x,y+side*EARY,z),2.25,80,'X'))
        if trap: cut(o,cyl('M4 nut',(x+sign*11,y+side*EARY,z),7.3/math.sqrt(3),4,'X',6))
    return o

def liner(diameter):
    o=cyl('Casquillo '+str(diameter),(0,0,0),21.2,23)
    cut(o,cyl('bore',(0,0,0),(diameter+.6)/2,30))
    cut(o,box('split',(15,0,0),(33,100,30)))
    return o

def make_main(c,phone):
    x,y=c['pole_offset_x_mm'],c['pole_offset_y_mm']
    px,py=phone.x+18.425,phone.y+10
    o=clamp('01 Cuerpo lateral',-1,(x,y,-95))
    join(o,clamp('Lower fixed clamp',-1,(x,y,-145)))
    join(o,box('Spine',(x-27.5,y,-105),(12,16,104)))
    join(o,box('MID-360 plate',(0,0,-50),(71,71,6)))
    join(o,beam('Lidar arm',(x-27.5,y,-63.85),(-24,0,-63.85),20,22.3))
    top=phone.z+24
    join(o,box('Phone riser',(px,py,(top-157)/2),(12,24,top+157)))
    for z in (-95,-145): join(o,beam('Phone arm',(x-27.5,y,z),(px,py,z),18,16))
    cut(o,cyl('Pole keepout',(x,y,-105),BORE,180))
    # Four 48 x 36 mm M3 axes; wells stop below the 6 mm plate.
    for hx in (-24,24):
        for hy in (-18,18):
            cut(o,cyl('M3 through',(hx,hy,-65),1.7,60))
            cut(o,cyl('M3 head and tool access',(hx,hy,-104),3.8,102))
    for z in (phone.z-52,phone.z+5):
        cut(o,cyl('M4 cradle mount',(px,py,z),2.25,30,'X'))
        cut(o,cyl('M4 nut',(px+5,py,z),7.3/math.sqrt(3),4,'X',6))
    bake(o)
    for hx in (-24,24):
        for hy in (-18,18):
            if o.ray_cast(Vector((hx,hy,-180)),Vector((0,0,1)),distance=145)[0]: raise RuntimeError('Blocked M3 axis')
            hit,p,_,_=o.ray_cast(Vector((hx+3,hy,-180)),Vector((0,0,1)),distance=145)
            if not hit or abs(p.z+53)>.05: raise RuntimeError('Blocked M3 head access')
    return o

def phone_parts(w,h,t,transform):
    half=w/2; gap=.4; front=-t/2-gap; mid=front-4; rear=front-8
    o=box('03 Cuna iPhone 14 Pro',(0,mid,-27),(w+12,8,104))
    floor=-h/2-gap-3
    join(o,box('Connected bottom stops',(0,-2,floor),(w+12,20,6)))
    cut(o,box('Charging cable opening',(0,-.3,floor),(20,15,10)))
    sx=half+gap+2.5
    join(o,box('Fixed side',(sx,-2,-30),(5,22,44)))
    join(o,box('Front lip',(half+gap-.5,t/2+2,-30),(11,3.2,44)))
    cut(o,box('Moving jaw clearance',(-43,mid,-30),(19,25,46)))
    for z in (-42,-18):
        cut(o,slot('M4 sliding slot',-25,-16,mid,z,2.25,20))
        cut(o,slot('M4 head recess',-25,-16,front-1.65,z,4.1,5.3))
    for z in (-52,5):
        cut(o,cyl('M4 main mount',(10,mid,z),2.25,20,'Y'))
        cut(o,cyl('M4 head recess',(10,front-1.65,z),4.1,5.3,'Y'))
    jaw=box('04 Mordaza movil',(-sx,-2.5,-30),(5,23,44))
    join(jaw,box('Front lip',(-half-gap+.5,t/2+2,-30),(11,3.2,44)))
    join(jaw,box('Sliding flange',(-25.5,rear-2.25,-30),(35,4,44)))
    for z in (-42,-18):
        cut(jaw,cyl('M4 slider',(-16,rear-2.25,z),2.25,40,'Y'))
        cut(jaw,cyl('M4 captive nut',(-16,rear-2.65,z),7.3/math.sqrt(3),3.6,'Y',6))
    rot=Matrix.Rotation(math.pi/2,3,'X')
    record(o,'03_cuna_iphone14pro_x1.stl',1,'PETG/ASA',rot)
    record(jaw,'04_mordaza_movil_x1.stl',1,'PETG/ASA',rot)
    for part in (o,jaw): part.matrix_world=transform

def preview():
    scene=bpy.context.scene; scene.unit_settings.system='METRIC'; scene.unit_settings.scale_length=.001
    mat=bpy.data.materials.new('Rig PETG'); mat.diffuse_color=(.05,.35,.42,1); mat.use_nodes=True
    bs=mat.node_tree.nodes.get('Principled BSDF'); bs.inputs['Base Color'].default_value=(.05,.35,.42,1); bs.inputs['Roughness'].default_value=.38
    for o in PARTS: o.data.materials.append(mat)
    bpy.ops.object.camera_add(location=(330,-460,280)); cam=bpy.context.object
    cam.rotation_euler=(Vector((-4,50,-40))-cam.location).to_track_quat('-Z','Y').to_euler()
    cam.data.type='ORTHO'; cam.data.ortho_scale=320; scene.camera=cam
    for pos,power in [((160,-250,330),1800000),((-230,180,200),1400000),((180,260,80),1000000)]:
        bpy.ops.object.light_add(type='AREA',location=pos); l=bpy.context.object; l.data.energy=power; l.data.size=200
        l.rotation_euler=(Vector((0,50,-50))-l.location).to_track_quat('-Z','Y').to_euler()
    scene.world.color=(.08,.08,.08); scene.render.engine='CYCLES'; scene.cycles.samples=32
    scene.render.resolution_x=1400; scene.render.resolution_y=1400; scene.render.resolution_percentage=100
    for screen in bpy.data.screens:
        for area in screen.areas:
            if area.type=='VIEW_3D':
                area.spaces.active.region_3d.view_distance=340; area.spaces.active.region_3d.view_location=Vector((-4,50,-40))
                area.spaces.active.region_3d.view_rotation=cam.rotation_euler.to_quaternion(); area.spaces.active.shading.color_type='MATERIAL'
    scene.render.filepath=str(OUT/'rig-sin-equipos.png'); scene.render.image_settings.file_format='PNG'
    bpy.context.preferences.filepaths.save_version=0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'rig-impresion.blend'))
    bpy.ops.render.render(write_still=True)

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    bpy.ops.object.select_all(action='SELECT'); bpy.ops.object.delete(use_global=False)
    c=json.loads((HERE/'rig_config.json').read_text())
    profiles=json.loads((HERE.parents[1]/'apps/ios/ATLASViewer/ATLASViewer/Resources/iphone-mid360-calibrations.json').read_text())
    p=next(p for p in profiles['profiles'] if p['id']==c['profile_id']); d=p['dimensions_mm']; w,h,t=d['width'],d['height'],d['thickness']
    r=Matrix([p['lidar_to_camera']['rotation_matrix_row_major'][i:i+3] for i in (0,3,6)])
    camera=r.transposed().to_4x4(); camera.translation=-(r.transposed()@Vector(p['lidar_to_camera']['translation_m'])*1000)+Vector((-16,60,0))
    body=camera@Matrix(((0,0,-1,0),(-1,0,0,0),(0,1,0,0),(0,0,0,1)))
    body.translation=camera.translation-body.to_3x3()@Vector((-w/2+c['camera_left_mm'],-t/2-c['camera_protrusion_mm'],h/2-c['camera_top_mm']))
    record(make_main(c,body.translation),'01_cuerpo_lateral_x1.stl',1,'PETG/ASA')
    cap=clamp('02 Tapa abrazadera',1,(0,0,0),True)
    record(cap,'02_tapa_abrazadera_x2.stl',2,'PETG/ASA')
    cap.location=(c['pole_offset_x_mm'],c['pole_offset_y_mm'],-95)
    dup=cap.copy(); dup.data=cap.data.copy(); bpy.context.collection.objects.link(dup); dup.location.z=-145
    phone_parts(w,h,t,body)
    for diameter in (35,30,40):
        o=liner(diameter); filename=('' if diameter==35 else 'opcionales/')+f'05_casquillo_{diameter}mm_x4.stl'
        record(o,filename,4,'TPU')
        if diameter!=35: o.hide_render=True; o.hide_set(True)
        else:
            o.location=(c['pole_offset_x_mm'],c['pole_offset_y_mm'],-95)
            for sign,z in [(1,-95),(0,-145),(1,-145)]:
                dup=o.copy(); dup.data=o.data.copy(); bpy.context.collection.objects.link(dup); dup.location.z=z
                if sign: dup.rotation_euler.z=math.pi
    (OUT/'piezas.json').write_text(json.dumps({'units':'mm','default_pole_diameter_mm':35,'parts':MANIFEST,'excluded':['pole','iPhone','MID-360','GNSS','metal fasteners'],'physical_prototype':True,'phone_shift_from_seed_mm':[-16,60,0]},indent=2)+'\n')
    inv=camera.inverted(); p['lidar_to_camera']={'translation_m':[round(v/1000,9) for v in inv.translation], 'rotation_matrix_row_major':[round(inv[i][j],9) for i in range(3) for j in range(3)]}; p['verified']=False
    p['notes']='Variante STL: teléfono -16 mm X, +60 mm Y para liberar M12. Verificar y recalibrar.'
    (OUT/'calibracion-variante-impresa.json').write_text(json.dumps({'schema_version':1,'profiles':[p]},ensure_ascii=False,indent=2)+'\n')
    preview()
if __name__=='__main__': main()
