"""ATLAS adjustable mounting study. Blender 4.2+ / 5.x; all geometry in mm.
Run: blender -b --python build_rig.py -- --output /absolute/folder [--render]
Inside the saved blend: Text Editor > ATLAS_RIG.py > Run Script, then N > ATLAS.
"""
import argparse
import json
import math
import sys
from pathlib import Path
import bpy
from mathutils import Matrix, Vector, Euler

HERE = Path(__file__).resolve().parent if '__file__' in globals() else Path(bpy.data.filepath).parent
CATALOG_PATH = HERE.parents[1] / 'apps/ios/ATLASViewer/ATLASViewer/Resources/iphone-mid360-calibrations.json'
COLLECTIONS = ('01_REFERENCIAS', '02_ABRAZADERAS', '03_BASE_LIDAR', '04_CUNA_TELEFONO', '05_TORNILLERIA', '06_MARCOS', '07_PRESENTACION')
MAT = {}

def text_json(name, fallback):
    text = bpy.data.texts.get(name)
    return json.loads(text.as_string()) if text else fallback()

def catalog():
    return text_json('phone_profiles.json', lambda: json.loads(CATALOG_PATH.read_text()))

def config():
    return text_json('rig_config.json', lambda: json.loads((HERE / 'rig_config.json').read_text()))

def material(name, rgb, metallic=0):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.diffuse_color = (*rgb, 1); m.use_nodes = True
    bs = m.node_tree.nodes.get('Principled BSDF')
    bs.inputs['Base Color'].default_value = (*rgb, 1)
    bs.inputs['Metallic'].default_value = metallic; bs.inputs['Roughness'].default_value = .32
    MAT[name] = m
    return m

def move(obj, group, mat=None):
    for c in list(obj.users_collection): c.objects.unlink(obj)
    bpy.data.collections[group].objects.link(obj)
    if mat: obj.data.materials.append(MAT[mat])
    return obj

def box(name, location, size, group, mat='structure', bevel=1):
    bpy.ops.mesh.primitive_cube_add(size=1, location=location)
    o = bpy.context.object; o.name = name; o.dimensions = size
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    move(o, group, mat)
    if bevel:
        mod = o.modifiers.new('Bordes mecanizados', 'BEVEL'); mod.width = bevel; mod.segments = 3
        mod = o.modifiers.new('Normales', 'WEIGHTED_NORMAL')
    return o

def cyl(name, location, radius, depth, group, mat='metal', axis='Z'):
    bpy.ops.mesh.primitive_cylinder_add(vertices=64, radius=radius, depth=depth, location=location)
    o = bpy.context.object; o.name = name
    if axis == 'X': o.rotation_euler.y = math.pi / 2
    if axis == 'Y': o.rotation_euler.x = math.pi / 2
    move(o, group, mat)
    for p in o.data.polygons: p.use_smooth = len(p.vertices) == 4
    return o

def subtract(obj, cutter):
    mod = obj.modifiers.new('Mecanizado', 'BOOLEAN'); mod.operation = 'DIFFERENCE'; mod.solver = 'EXACT'; mod.object = cutter
    # Apply before the cosmetic bevels, preserving holes and actual slot topology.
    bpy.context.view_layer.objects.active = obj
    while list(obj.modifiers).index(mod) > 0: bpy.ops.object.modifier_move_up(modifier=mod.name)
    bpy.ops.object.modifier_apply(modifier=mod.name)
    bpy.data.objects.remove(cutter, do_unlink=True)

def empty(name, matrix=None, size=15):
    o = bpy.data.objects.new(name, None); bpy.data.collections['06_MARCOS'].objects.link(o)
    o.empty_display_type = 'ARROWS'; o.empty_display_size = size
    if matrix is not None: o.matrix_world = matrix
    return o

def label(name, text, pos, size=7, rotation=(math.pi/2, 0, math.pi/2)):
    curve = bpy.data.curves.new(name, 'FONT'); curve.body = text; curve.size = size; curve.extrude = .03
    o = bpy.data.objects.new(name, curve); bpy.data.collections['07_PRESENTACION'].objects.link(o)
    o.location = pos; o.rotation_euler = rotation; o.data.materials.append(MAT['text'])
    return o

def ring_half(name, z, inner, outer, sign, group, mat, center, depth=22):
    x,y=center
    o = cyl(name, (x,y,z), outer, depth, group, mat)
    subtract(o, cyl('Corte interior', (x,y,z), inner, depth+4, group))
    subtract(o, box('Separacion', (x-sign*outer/2,y,z), (outer+2,outer*3,depth+4),group,bevel=0))
    return o

def local(obj, parent):
    obj.parent = parent
    return obj

def generate(c, profiles):
    # Only replace this generator's collections; preserve unrelated user objects.
    for name in COLLECTIONS:
        collection = bpy.data.collections.get(name)
        if collection:
            for o in list(collection.objects):
                if o.type not in ('CAMERA','LIGHT'): bpy.data.objects.remove(o, do_unlink=True)
        else:
            collection = bpy.data.collections.new(name); bpy.context.scene.collection.children.link(collection)
    for name, rgb, metallic in [('structure',(.045,.22,.28),.55),('metal',(.42,.49,.53),.8),('rubber',(.022,.028,.034),0),
            ('phone',(.20,.18,.29),.55),('glass',(.025,.055,.068),.65),('accent',(.96,.42,.08),.4),('text',(.65,.83,.87),.1)]:
        material(name,rgb,metallic)
    p = next(p for p in profiles['profiles'] if p['id'] == c['profile_id'])
    d = p['dimensions_mm']; w,h,t = d['width'],d['height'],d['thickness']
    if min(w,h,t) <= 0 or not 30 <= c['pole_diameter_mm'] <= 40: raise ValueError('Elige un teléfono con cotas y bastón entre 30 y 40 mm')
    root = empty('ATLAS_PARAMETERS')
    root['profile_id'] = c['profile_id']; root['phone_width_mm']=w; root['phone_height_mm']=h; root['phone_thickness_mm']=t
    for key,value in c.items():
        if isinstance(value,(str,float,int,bool)): root[key] = value
    root.id_properties_ui('pole_diameter_mm').update(min=30,max=40)
    for key in ('camera_left_mm','camera_top_mm','camera_protrusion_mm','phone_case_extra_mm'):
        root.id_properties_ui(key).update(min=0,max=100)
    root['IMPORTANTE'] = 'Editar propiedades > N > ATLAS > Actualizar. Centro óptico dentro del teléfono pendiente de medir.'
    scene = bpy.context.scene; scene.unit_settings.system = 'METRIC'; scene.unit_settings.scale_length = .001; scene.unit_settings.length_unit = 'MILLIMETERS'
    r = Matrix([p['lidar_to_camera']['rotation_matrix_row_major'][i:i+3] for i in (0,3,6)])
    tr = Vector(p['lidar_to_camera']['translation_m']) * 1000
    lidar_from_camera = r.transposed().to_4x4(); lidar_from_camera.translation = -(r.transposed() @ tr)
    delta = Euler(tuple(math.radians(c['phone_'+axis+'_deg']) for axis in ('roll','pitch','yaw')), 'XYZ').to_matrix().to_4x4()
    camera_matrix = delta @ lidar_from_camera
    camera_matrix.translation = lidar_from_camera.translation + Vector(tuple(c['phone_shift_'+a+'_mm'] for a in 'xyz'))
    camera = empty('CAMERA_OPTICAL_FRAME', camera_matrix, 25); camera['calibration_verified'] = False
    camera['convencion'] = '+X derecha, +Y arriba, -Z óptico; imagen nativa ARKit'
    empty('LIDAR_FRAME', Matrix.Identity(4), 35)
    native_from_body = Matrix(((0,0,-1),(-1,0,0),(0,1,0))).to_4x4()
    anchor = Vector((-w/2+c['camera_left_mm'], -t/2-c['camera_protrusion_mm'], h/2-c['camera_top_mm']))
    body_matrix = camera_matrix @ native_from_body
    body_matrix.translation = camera_matrix.translation - body_matrix.to_3x3() @ anchor
    body = empty('PHONE_BODY_FRAME',body_matrix); body['camera_anchor_mm'] = list(anchor)
    local(box(p['display_name']+' • envolvente', (0,0,0),(w,t,h),'01_REFERENCIAS','phone',3),body)
    local(box('Pantalla', (0,t/2+.2,0),(w-4,.5,h-4),'01_REFERENCIAS','glass',1.5),body)
    local(box('Volumen de cámaras • aproximado',(-w/2+20,-t/2-1,h/2-20),(38,2,38),'01_REFERENCIAS','phone',3),body)
    for name,x,z in [('Lente activa • centro editable',anchor.x,anchor.z),('Lente secundaria A',-w/2+28,h/2-12),('Lente secundaria B',-w/2+20,h/2-28)]:
        local(cyl(name,(x,anchor.y,z),5.7,2,'01_REFERENCIAS','glass','Y'),body)
    # Camera frustum: edges are explicit guides, not optical calibration measurements.
    for x,y in [(-25,-18),(25,-18),(25,18),(-25,18)]:
        mesh = bpy.data.meshes.new('Rayo RGB'); mesh.from_pydata([(0,0,0),(x,y,-70)],[(0,1)],[])
        o=bpy.data.objects.new('RGB • guía de visión',mesh); bpy.data.collections['06_MARCOS'].objects.link(o); o.parent=camera
    base_z = -c['lidar_origin_above_base_mm']; plate_z=base_z-c['plate_thickness_mm']/2
    width = c['mid360_width_mm']; height=c['mid360_height_mm']
    box('MID-360 • cuerpo nominal',(0,0,base_z+19.75),(width,width,39.5),'01_REFERENCIAS','metal',4)
    cyl('MID-360 • aro óptico',(0,0,base_z+39.5),25.5,4,'01_REFERENCIAS','rubber')
    bpy.ops.mesh.primitive_uv_sphere_add(segments=48,ring_count=24,radius=1,location=(0,0,base_z+49.75))
    dome=bpy.context.object; dome.name='MID-360 • ventana óptica simplificada'; dome.scale=(22,22,(height-39.5)/2); move(dome,'01_REFERENCIAS','glass')
    for face in dome.data.polygons: face.use_smooth = True
    cyl('M12 • conector (-X)',(-width/2-4,0,base_z+14.3),6,8,'01_REFERENCIAS','metal','X')
    # Four actual clearances: 48 x 36 mm, M3 thread depth in the sensor 5 mm.
    plate=box('Base MID-360 • 4 M3',(0,0,plate_z),(width+6,width+6,c['plate_thickness_mm']),'03_BASE_LIDAR')
    for x in (-c['mid360_mount_spacing_x_mm']/2,c['mid360_mount_spacing_x_mm']/2):
        for y in (-c['mid360_mount_spacing_y_mm']/2,c['mid360_mount_spacing_y_mm']/2):
            subtract(plate,cyl('Taladro M3',(x,y,plate_z),1.7,14,'03_BASE_LIDAR'))
            cyl('M3 • cabeza bajo placa',(x,y,base_z-c['plate_thickness_mm']-1.5),2.8,3,'05_TORNILLERIA')
    mast_x,mast_y=c['pole_offset_x_mm'],c['pole_offset_y_mm']
    pole_top=c['pole_top_above_lidar_mm']
    cyl('GNSS • volumen reservado, dimensiones por confirmar',(mast_x,mast_y,pole_top+c['gnss_reserved_height_mm']/2),c['gnss_reserved_diameter_mm']/2,c['gnss_reserved_height_mm'],'01_REFERENCIAS','accent')
    empty('GNSS_RESERVED_FRAME',Matrix.Translation(Vector((mast_x,mast_y,pole_top))),30)
    cyl('Bastón • diámetro editable',(mast_x,mast_y,pole_top-c['pole_length_mm']/2),c['pole_diameter_mm']/2,c['pole_length_mm'],'01_REFERENCIAS','rubber')
    bore=c['clamp_bore_mm']/2; outer=bore+5
    for z in (base_z-48,base_z-98):
        for sign in (-1,1):
            ring_half('Abrazadera partida',z,bore,outer,sign,'02_ABRAZADERAS','structure',(mast_x,mast_y))
            ring_half('Casquillo para '+str(c['pole_diameter_mm'])+' mm',z,(c['pole_diameter_mm']+c['fit_clearance_mm'])/2,bore-.1,sign,'02_ABRAZADERAS','accent',(mast_x,mast_y))
            for y in (-outer,outer):
                tab=box('Oreja de apriete',(mast_x+sign*5,mast_y+y,z),(8,11,22),'02_ABRAZADERAS',bevel=.7)
                subtract(tab,cyl('Paso M4',(mast_x+sign*5,mast_y+y,z),2.2,12,'02_ABRAZADERAS',axis='X'))
        for y in (-outer,outer):
            cyl('M4 • tornillo abrazadera',(mast_x,mast_y+y,z),2,26,'05_TORNILLERIA',axis='X')
            cyl('M4 • pomo de ajuste',(mast_x-17,mast_y+y,z),5,6,'05_TORNILLERIA','accent','X')
    box('Pilar lateral base LiDAR',(mast_x+outer+5,mast_y,base_z-29),(12,28,46),'03_BASE_LIDAR')
    start=Vector((mast_x+outer+5,mast_y,base_z-12)); end=Vector((width/2-9,0,base_z-12))
    arm=box('Brazo lateral • base MID-360',(start+end)/2,((end-start).length+14,16,12),'03_BASE_LIDAR')
    arm.rotation_euler.z=math.atan2(end.y-start.y,end.x-start.x)
    # Phone cradle in its own frame: back plate, two sliding jaws and lower stops.
    extra=c['phone_case_extra_mm']; clearance=c['fit_clearance_mm']; grip_w=w+2*extra+clearance
    back_y=t/2+extra+4
    cradle=local(box('Cuna • placa posterior ranurada',(0,back_y,-24),(grip_w+16,6,90),'04_CUNA_TELEFONO'),body)
    for x in (-22,22):
        cut=local(box('Ranura vertical',(x,back_y,-24),(5,12,56),'04_CUNA_TELEFONO',bevel=0),body)
        bpy.context.view_layer.update(); subtract(cradle,cut)
    for side in (-1,1):
        local(box('Mordaza lateral ajustable',(side*(grip_w/2+3),0,-24),(5,t+2*extra+12,30),'04_CUNA_TELEFONO','structure'),body)
        local(box('Almohadilla lateral',(side*(grip_w/2+.7),0,-24),(1.5,t+2*extra,28),'04_CUNA_TELEFONO','rubber',.4),body)
        local(box('Labio de retención',(side*(grip_w/2-1),-t/2-extra-2,-24),(9,3,26),'04_CUNA_TELEFONO','accent',.5),body)
        local(box('Tope inferior • USB libre',(side*22,0,-h/2-extra-3),(14,t+2*extra+10,5),'04_CUNA_TELEFONO'),body)
        local(cyl('Pomo M4 • ancho',(side*(grip_w/2+4),back_y+6,-24),5,6,'05_TORNILLERIA','accent','Y'),body)
    # Two horizontal carriage channels; slots are usable machining geometry.
    for z in (-42,-8):
        rail=local(box('Corredera transversal',(0,back_y+6,z),(grip_w+20,6,12),'04_CUNA_TELEFONO'),body)
        cut=local(box('Ranura transversal',(0,back_y+6,z),(grip_w-12,12,4.5),'04_CUNA_TELEFONO',bevel=0),body)
        bpy.context.view_layer.update(); subtract(rail,cut)
    bpy.context.view_layer.update()
    attach=body.matrix_world @ Vector((0,back_y+10,-24))
    rail_bottom=min(base_z-108,attach.z-55); rail_top=max(base_z-40,attach.z+42)
    rail=box('Montante • ajuste de altura',(attach.x,attach.y,(rail_top+rail_bottom)/2),(8,24,rail_top-rail_bottom),'04_CUNA_TELEFONO')
    subtract(rail,box('Ranura de altura',(attach.x,attach.y,(rail_top+rail_bottom)/2),(14,5,rail_top-rail_bottom-22),'04_CUNA_TELEFONO',bevel=0))
    for z in (base_z-48,base_z-98):
        delta=Vector((attach.x-mast_x,attach.y-mast_y,0)).normalized()*(outer+3)
        start=Vector((mast_x,mast_y,z))+delta; end=Vector((attach.x,attach.y,z))
        beam=box('Travesaño bastón-cuna',(start+end)/2,((end-start).length+8,18,12),'04_CUNA_TELEFONO')
        beam.rotation_euler.z=math.atan2(end.y-start.y,end.x-start.x)
        cyl('Pomo M4 • altura',(attach.x-8,attach.y,z),5,7,'05_TORNILLERIA','accent','X')
    # Diagnostic connector penetration in the configured phone envelope.
    connector=Vector((-width/2-4,0,base_z+14.3)); q=body.matrix_world.inverted() @ connector
    collision=abs(q.x)<w/2+6 and abs(q.y)<t/2+6 and abs(q.z)<h/2+6
    root['connector_clearance_review'] = bool(collision)
    root['LIDAR_FOV'] = 'Teléfono por encima del plano óptico: revisar sector de oclusión trasero; no se promete FOV libre de 360°.'
    root['manufacturing_status'] = 'Estudio de montaje editable; validar cable, rigidez, recorridos y tornillería antes de fabricar.'
    label('Título','ATLAS / RIG LATERAL',(-75,-75,113),10)
    label('Subtítulo',p['display_name']+' + MID-360\nBastón '+str(c['pole_diameter_mm'])+' mm  |  ajuste 30–40 mm\nPunta libre para GNSS • conjunto a un lado',(-75,-75,96),4.5)
    label('Nota','Calibración nominal · centro óptico editable\n'+('REVISAR INTERFERENCIA M12 / TELÉFONO' if collision else 'Revisar recorrido real del cable M12'),(-75,-75,-184),4)
    scene['rig_config']=json.dumps(c,ensure_ascii=False)
    scene['fit_report']=json.dumps({'connector_interference':bool(collision),'camera_origin_in_lidar_mm':list(camera_matrix.translation),'phone_body_center_in_lidar_mm':list(body_matrix.translation),'calibration_verified':False},indent=2)
    bpy.ops.object.select_all(action='DESELECT'); root.select_set(True); bpy.context.view_layer.objects.active=root
    return root

def export_calibration(path):
    root=bpy.data.objects['ATLAS_PARAMETERS']; p=next(p.copy() for p in catalog()['profiles'] if p['id']==root['profile_id'])
    m=bpy.data.objects['CAMERA_OPTICAL_FRAME'].matrix_world.inverted()
    p['lidar_to_camera']={'translation_m':[float(v)/1000 for v in m.translation], 'rotation_matrix_row_major':[float(m[i][j]) for i in range(3) for j in range(3)]}
    p['verified']=False; p['camera_lens']='wide'; p['notes']='Propuesta desde Blender; medir y validar reproyección antes de marcar verified.'
    Path(path).write_text(json.dumps({'schema_version':1,'profiles':[p]},ensure_ascii=False,indent=2)+'\n')

class ATLAS_OT_rebuild(bpy.types.Operator):
    bl_idname='atlas.rebuild_rig'; bl_label='Actualizar rig desde parámetros'
    def execute(self, context):
        c=config(); root=bpy.data.objects.get('ATLAS_PARAMETERS')
        if root:
            for key in c:
                if key in root: c[key]=root[key]
        try:
            generate(c,catalog())
            camera=context.scene.camera
            if camera:
                for name,pos in [('Título',(-285,245,-600)),('Subtítulo',(-285,223,-600)),('Nota',(-285,-240,-600))]:
                    o=bpy.data.objects[name]; o.parent=camera; o.location=pos; o.rotation_euler=(0,0,0)
        except Exception as e: self.report({'ERROR'},str(e)); return {'CANCELLED'}
        return {'FINISHED'}

class ATLAS_OT_profile(bpy.types.Operator):
    bl_idname='atlas.select_profile'; bl_label='Elegir teléfono'
    profile: bpy.props.EnumProperty(items=[(p['id'],p['display_name'],'') for p in catalog()['profiles'] if p['dimensions_mm']['width']>0])
    def execute(self,context):
        bpy.data.objects['ATLAS_PARAMETERS']['profile_id']=self.profile
        return bpy.ops.atlas.rebuild_rig()

class ATLAS_OT_export(bpy.types.Operator):
    bl_idname='atlas.export_calibration'; bl_label='Exportar calibración propuesta'
    def execute(self,context):
        path=Path(bpy.data.filepath).parent/'calibration-proposed.json'; export_calibration(path)
        self.report({'INFO'},str(path)); return {'FINISHED'}

class ATLAS_PT_rig(bpy.types.Panel):
    bl_label='ATLAS · Rig ajustable'; bl_idname='ATLAS_PT_rig'; bl_space_type='VIEW_3D'; bl_region_type='UI'; bl_category='ATLAS'
    def draw(self,context):
        layout=self.layout; root=bpy.data.objects.get('ATLAS_PARAMETERS')
        if root:
            layout.operator_menu_enum('atlas.select_profile','profile',text='Cambiar modelo de teléfono')
            layout.label(text=f"{root['phone_height_mm']} × {root['phone_width_mm']} × {root['phone_thickness_mm']} mm")
            for key in ('profile_id','pole_diameter_mm','pole_offset_x_mm','pole_offset_y_mm','pole_top_above_lidar_mm','phone_case_extra_mm','camera_left_mm','camera_top_mm','camera_protrusion_mm','phone_shift_x_mm','phone_shift_y_mm','phone_shift_z_mm','phone_roll_deg','phone_pitch_deg','phone_yaw_deg'):
                layout.prop(root,'["'+key+'"]',text=key)
        layout.operator('atlas.rebuild_rig'); layout.operator('atlas.export_calibration')
        layout.label(text='Centro óptico y montaje: verificar físicamente')

for cls in (ATLAS_OT_rebuild,ATLAS_OT_profile,ATLAS_OT_export,ATLAS_PT_rig):
    old=getattr(bpy.types,cls.__name__,None)
    if old: bpy.utils.unregister_class(old)
    bpy.utils.register_class(cls)

def presentation(output, render):
    scene=bpy.context.scene; scene.render.engine='CYCLES'; scene.cycles.samples=32
    scene.render.resolution_x=1800; scene.render.resolution_y=1600; scene.render.resolution_percentage=100
    scene.world.color=(.12,.12,.12)
    bpy.ops.object.camera_add(location=(460,-650,330)); camera=bpy.context.object; move(camera,'07_PRESENTACION')
    camera.rotation_euler=(Vector((0,45,40))-camera.location).to_track_quat('-Z','Y').to_euler()
    camera.data.type='ORTHO'; camera.data.ortho_scale=620; scene.camera=camera
    for name,pos in [('Título',(-285,245,-600)),('Subtítulo',(-285,223,-600)),('Nota',(-285,-240,-600))]:
        o=bpy.data.objects[name]; o.parent=camera; o.location=pos; o.rotation_euler=(0,0,0); o.data.size *= 1.5
    # Text overlay stays legible independently of the viewing angle.
    mat=bpy.data.materials['text']; bs=mat.node_tree.nodes.get('Principled BSDF')
    bs.inputs['Emission Color'].default_value=(.65,.83,.87,1); bs.inputs['Emission Strength'].default_value=1
    for pos,power,size in [((-200,-240,350),1800000,250),((240,50,200),2200000,220),((-140,270,80),1400000,180)]:
        bpy.ops.object.light_add(type='AREA',location=pos); light=bpy.context.object; move(light,'07_PRESENTACION')
        light.data.energy=power; light.data.shape='DISK'; light.data.size=size
        light.rotation_euler=(Vector((0,0,-30))-light.location).to_track_quat('-Z','Y').to_euler()
    scene.world.use_nodes=True; scene.world.node_tree.nodes['Background'].inputs[0].default_value=(.025,.04,.06,1)
    scene.world.node_tree.nodes['Background'].inputs[1].default_value=.35
    scene.render.image_settings.file_format='PNG'; scene.render.filepath=str(output/'rig-preview.png')
    for screen in bpy.data.screens:
        for area in screen.areas:
            if area.type=='VIEW_3D':
                area.spaces.active.region_3d.view_distance=620
                area.spaces.active.region_3d.view_location=Vector((0,45,40))
                area.spaces.active.region_3d.view_rotation=camera.rotation_euler.to_quaternion()
                area.spaces.active.shading.color_type='MATERIAL'; area.spaces.active.clip_end=10000
    bpy.context.preferences.filepaths.save_version=0
    bpy.ops.wm.save_as_mainfile(filepath=str(output/'ATLAS_iPhone14Pro_MID360_Rig.blend'))
    if render: bpy.ops.render.render(write_still=True)

if __name__ == '__main__' and bpy.app.background:
    parser=argparse.ArgumentParser(); parser.add_argument('--output',type=Path,default=HERE); parser.add_argument('--render',action='store_true')
    args=parser.parse_args(sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else [])
    args.output.mkdir(parents=True,exist_ok=True)
    profiles=catalog(); c=config()
    # Factory scene objects are unrelated to the rig and can be removed only for initial CLI generation.
    bpy.ops.object.select_all(action='SELECT'); bpy.ops.object.delete(use_global=False)
    generate(c,profiles)
    for name,content in [('phone_profiles.json',json.dumps(profiles)),('rig_config.json',json.dumps(c)),('ATLAS_RIG.py',Path(__file__).read_text())]:
        block=bpy.data.texts.get(name) or bpy.data.texts.new(name); block.clear(); block.write(content)
    (args.output/'fit-report.json').write_text(bpy.context.scene['fit_report']+'\n')
    export_calibration(args.output/'calibration-proposed.json')
    presentation(args.output,args.render)
