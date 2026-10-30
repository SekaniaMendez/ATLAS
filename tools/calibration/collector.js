'use strict';
const $ = id => document.getElementById(id);
let documentData, current, points, photo, selected = -1, uv = null, xyz = null, draftKey, loadGeneration = 0;
let yaw = 0, pitch = 0, center = [0, 0, 0], radius = 1, cloudScale = 100;
let photoScale = 1, photoX = 0, photoY = 0;
const photoCanvas = $('photo'), cloudCanvas = $('cloud'), imageContext = photoCanvas.getContext('2d');
const gl = cloudCanvas.getContext('webgl', {antialias: true});
function status(text) { $('status').textContent = text; }
function compile(type, text) {
  const shader = gl.createShader(type); gl.shaderSource(shader, text); gl.compileShader(shader);
  if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) throw Error(gl.getShaderInfoLog(shader));
  return shader;
}
let program, buffer, uniformLocations;
function initializeGL() {
  if (!gl) throw Error('Este navegador no ofrece WebGL.');
  program = gl.createProgram();
  gl.attachShader(program, compile(gl.VERTEX_SHADER, `attribute vec3 position;
    uniform vec3 center, right, up, eye; uniform vec2 viewport;
    uniform float scale, radius, size; varying float shade;
    void main(){vec3 p=position-center; float depth=dot(p,eye);
      gl_Position=vec4(2.0*dot(p,right)*scale/viewport.x,2.0*dot(p,up)*scale/viewport.y,-depth/radius,1.0);
      gl_PointSize=size; shade=clamp(0.6+depth/radius*0.4,0.3,1.0);}`));
  gl.attachShader(program, compile(gl.FRAGMENT_SHADER, `precision mediump float; varying float shade;
    uniform vec3 color; void main(){gl_FragColor=vec4(color*shade,1.0);}`));
  gl.linkProgram(program);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw Error(gl.getProgramInfoLog(program));
  gl.useProgram(program); buffer = gl.createBuffer();
  uniformLocations = Object.fromEntries(['center','right','up','eye','viewport','scale','radius','size','color'].map(k=>[k,gl.getUniformLocation(program,k)]));
  gl.enableVertexAttribArray(gl.getAttribLocation(program, 'position'));
  gl.enable(gl.DEPTH_TEST); gl.clearColor(0.06,0.10,0.13,1);
}
function basis() { return {right:[-Math.sin(yaw),Math.cos(yaw),0],
  up:[-Math.sin(pitch)*Math.cos(yaw),-Math.sin(pitch)*Math.sin(yaw),Math.cos(pitch)],
  eye:[Math.cos(pitch)*Math.cos(yaw),Math.cos(pitch)*Math.sin(yaw),Math.sin(pitch)]}; }
const dot = (a,b) => a.reduce((s,x,i)=>s+x*b[i],0);
function drawCloud() {
  if (!points || !program) return;
  gl.viewport(0,0,cloudCanvas.width,cloudCanvas.height); gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
  const b = basis(); gl.uniform3fv(uniformLocations.center,center);
  for (const k of ['right','up','eye']) gl.uniform3fv(uniformLocations[k],b[k]);
  gl.uniform2f(uniformLocations.viewport,cloudCanvas.width,cloudCanvas.height);
  gl.uniform1f(uniformLocations.scale,cloudScale); gl.uniform1f(uniformLocations.radius,radius*2);
  gl.uniform1f(uniformLocations.size,Number($('size').value)); gl.uniform3f(uniformLocations.color,0.55,0.85,0.88);
  gl.bindBuffer(gl.ARRAY_BUFFER,buffer); gl.bufferData(gl.ARRAY_BUFFER,points,gl.STATIC_DRAW);
  gl.vertexAttribPointer(gl.getAttribLocation(program,'position'),3,gl.FLOAT,false,0,0);
  gl.drawArrays(gl.POINTS,0,points.length/3);
  const marks = current.correspondences.map(p=>p.xyz_lidar_m);
  if (xyz) marks.push(xyz);
  if (marks.length) {
    gl.disable(gl.DEPTH_TEST); gl.uniform1f(uniformLocations.size,8); gl.uniform3f(uniformLocations.color,1,0.65,0.2);
    gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(marks.flat()),gl.STREAM_DRAW);
    gl.drawArrays(gl.POINTS,0,marks.length); gl.enable(gl.DEPTH_TEST);
  }
}
function drawPhoto() {
  const c=imageContext; c.clearRect(0,0,photoCanvas.width,photoCanvas.height);
  if (!photo) return;
  c.drawImage(photo,photoX,photoY,photo.width*photoScale,photo.height*photoScale);
  function mark(p,label,color) {const x=photoX+p[0]*photoScale,y=photoY+p[1]*photoScale;
    c.strokeStyle=color;c.lineWidth=2;c.beginPath();c.arc(x,y,5,0,2*Math.PI);c.stroke();
    c.fillStyle=color;c.font='bold 14px system-ui';c.fillText(label,x+8,y-8);}
  current.correspondences.forEach((p,i)=>mark(p.uv_px,String(i+1),i===selected?'#fff':'#ffb954'));
  if (uv) mark(uv,'+', '#fff');
}
function fitPhoto() {
  if (!photo) return;
  photoScale=Math.min(photoCanvas.width/photo.width,photoCanvas.height/photo.height)*0.96;
  photoX=(photoCanvas.width-photo.width*photoScale)/2;photoY=(photoCanvas.height-photo.height*photoScale)/2;drawPhoto();
}
function fitCloud() {
  if (!points) return;
  const min=[Infinity,Infinity,Infinity],max=[-Infinity,-Infinity,-Infinity];
  for(let i=0;i<points.length;i++) {const k=i%3;min[k]=Math.min(min[k],points[i]);max[k]=Math.max(max[k],points[i]);}
  center=min.map((x,k)=>(x+max[k])/2);radius=Math.max(0.1,Math.hypot(...max.map((x,k)=>x-min[k]))/2);
  cloudScale=Math.min(cloudCanvas.width,cloudCanvas.height)/radius*0.44;drawCloud();
}
function saveDraft() {
  try {localStorage.setItem(draftKey,JSON.stringify(documentData));status('Borrador guardado en este navegador. Descárgalo para procesarlo.');}
  catch {status('No se pudo guardar el borrador local. Descarga las correspondencias.');}
}
function updatePairs() {
  $('add').disabled=!(uv&&xyz);$('remove').disabled=selected<0;
  $('pending').textContent=`Imagen: ${uv?uv.map(n=>n.toFixed(1)).join(', ')+' px':'pendiente'} · LiDAR: ${xyz?xyz.map(n=>n.toFixed(4)).join(', ')+' m':'pendiente'}`;
  $('pairs').replaceChildren(...current.correspondences.map((p,i)=>{
    const button=document.createElement('button');button.textContent=`${i+1} · ${p.id}`;
    button.classList.toggle('active',selected===i);button.onclick=()=>{selected=i;updatePairs();};return button;
  }));drawPhoto();drawCloud();
}
async function setView(index) {
  const generation=++loadGeneration; current=documentData.views[index];uv=xyz=null;selected=-1;points=photo=null;
  $('split').textContent=current.split==='validation'?'Reservada para validar':'Para ajustar';
  $('static').checked=current.static_pair_confirmed===true;status('Cargando imagen y nube…');
  const [cloudResponse, image] = await Promise.all([fetch(current.cloud), new Promise((resolve,reject)=>{
    const image=new Image();image.onload=()=>resolve(image);image.onerror=()=>reject(Error('No se pudo abrir la imagen'));image.src=current.image;})]);
  if(!cloudResponse.ok) throw Error('No se pudo abrir la nube');
  const bytes=await cloudResponse.arrayBuffer();if(generation!==loadGeneration)return;
  points=new Float32Array(bytes);photo=image;
  if(points.length!==current.point_count*3 || photo.naturalWidth!==current.width || photo.naturalHeight!==current.height)
    throw Error('Las dimensiones no coinciden con el manifiesto.');
  yaw=0;pitch=0;fitPhoto();fitCloud();updatePairs();status(`${current.point_count.toLocaleString()} retornos. Revisa que sea la misma posición inmóvil.`);
}
function canvasPosition(event,canvas){const r=canvas.getBoundingClientRect();return [(event.clientX-r.left)*canvas.width/r.width,(event.clientY-r.top)*canvas.height/r.height];}
function interact(canvas,isPhoto) {
  let drag=null;
  canvas.addEventListener('pointerdown',event=>{if(!points||!photo)return;canvas.setPointerCapture(event.pointerId);
    const p=canvasPosition(event,canvas);drag={id:event.pointerId,start:p,last:p,moved:false,pan:event.shiftKey||event.button===2};});
  canvas.addEventListener('pointermove',event=>{
    if(!drag||drag.id!==event.pointerId)return;const p=canvasPosition(event,canvas);
    if(Math.hypot(p[0]-drag.start[0],p[1]-drag.start[1])>4)drag.moved=true;
    const dx=p[0]-drag.last[0],dy=p[1]-drag.last[1];drag.last=p;
    if(!drag.moved)return;
    if(isPhoto){photoX+=dx;photoY+=dy;drawPhoto();}
    else if(drag.pan){const b=basis();center=center.map((x,i)=>x-dx/cloudScale*b.right[i]+dy/cloudScale*b.up[i]);drawCloud();}
    else{yaw-=dx*0.008;pitch=Math.max(-1.56,Math.min(1.56,pitch+dy*0.008));drawCloud();}
  });
  canvas.addEventListener('pointerup',event=>{
    if(!drag||drag.id!==event.pointerId)return;const moved=drag.moved;drag=null;if(moved)return;
    const p=canvasPosition(event,canvas);
    if(isPhoto){const u=(p[0]-photoX)/photoScale,v=(p[1]-photoY)/photoScale;if(u<0||v<0||u>=photo.width||v>=photo.height)return;uv=[u,v];}
    else {const b=basis();let best=36,bestIndex=-1,depth=-Infinity;
      for(let i=0;i<points.length;i+=3){const delta=[points[i]-center[0],points[i+1]-center[1],points[i+2]-center[2]];
        const z=dot(delta,b.eye);if(Math.abs(z)>radius*2)continue;
        const x=cloudCanvas.width/2+dot(delta,b.right)*cloudScale,y=cloudCanvas.height/2-dot(delta,b.up)*cloudScale;
        const d=(x-p[0])**2+(y-p[1])**2;
        if(d<best-0.25||(d<=best+0.25&&d<=36&&z>depth)){best=d;bestIndex=i;depth=z;}}
      if(bestIndex<0){status('No hay un retorno suficientemente cerca. Amplía la nube.');return;}
      xyz=Array.from(points.slice(bestIndex,bestIndex+3));}
    updatePairs();
  });
  canvas.addEventListener('pointercancel',()=>{drag=null;});canvas.addEventListener('contextmenu',e=>e.preventDefault());
  canvas.addEventListener('wheel',event=>{event.preventDefault();const f=Math.exp(-Math.max(-100,Math.min(100,event.deltaY))*0.003);
    if(isPhoto){const p=canvasPosition(event,canvas),next=Math.max(0.02,Math.min(20,photoScale*f)),ratio=next/photoScale;photoX=p[0]-(p[0]-photoX)*ratio;photoY=p[1]-(p[1]-photoY)*ratio;photoScale=next;drawPhoto();}
    else{cloudScale=Math.max(0.1,Math.min(100000,cloudScale*f));drawCloud();}}, {passive:false});
}
function validDraft(other) {
  if(other.schema_version!==1||other.camera_lens!==documentData.camera_lens||!Array.isArray(other.views)||other.views.length!==documentData.views.length)return false;
  return other.views.every((v,i)=>{const base=documentData.views[i];return v.id===base.id&&v.image_sha256===base.image_sha256&&v.cloud_source?.source_sha256===base.cloud_source.source_sha256
    &&Array.isArray(v.correspondences)&&v.correspondences.length<=256&&v.correspondences.every(p=>typeof p.id==='string'&&p.id.length<=64&&Array.isArray(p.xyz_lidar_m)&&p.xyz_lidar_m.length===3&&p.xyz_lidar_m.every(Number.isFinite)
      &&Array.isArray(p.uv_px)&&p.uv_px.length===2&&p.uv_px.every(Number.isFinite)&&p.uv_px[0]>=0&&p.uv_px[0]<base.width&&p.uv_px[1]>=0&&p.uv_px[1]<base.height);});
}
function applyDraft(other) {documentData.views.forEach((v,i)=>{v.correspondences=other.views[i].correspondences;v.static_pair_confirmed=other.views[i].static_pair_confirmed===true;});}
async function main() {
  initializeGL();const response=await fetch('observations.json');if(!response.ok)throw Error('Falta observations.json');documentData=await response.json();
  draftKey='atlas.calibration.'+documentData.views.map(v=>v.image_sha256+v.cloud_source.source_sha256).join('.');
  try{const saved=JSON.parse(localStorage.getItem(draftKey));if(saved&&validDraft(saved))applyDraft(saved);}catch{}
  $('view').replaceChildren(...documentData.views.map((v,i)=>{const option=document.createElement('option');option.value=i;option.textContent=v.id;return option;}));
  $('view').onchange=()=>setView(Number($('view').value)).catch(e=>status(e.message));
  $('fitPhoto').onclick=fitPhoto;$('fitCloud').onclick=fitCloud;$('size').oninput=drawCloud;
  for(const [id,y,p]of[['front',0,0],['back',Math.PI,0],['top',0,Math.PI/2]])$(id).onclick=()=>{yaw=y;pitch=p;drawCloud();};
  $('clear').onclick=()=>{uv=xyz=null;selected=-1;updatePairs();};
  $('add').onclick=()=>{if(!uv||!xyz)return;current.correspondences.push({id:'corner-'+crypto.randomUUID().slice(0,8),xyz_lidar_m:xyz,uv_px:uv});uv=xyz=null;selected=-1;saveDraft();updatePairs();};
  $('remove').onclick=()=>{if(selected<0)return;current.correspondences.splice(selected,1);selected=-1;saveDraft();updatePairs();};
  $('static').onchange=()=>{current.static_pair_confirmed=$('static').checked;saveDraft();};
  $('download').onclick=()=>{const blob=new Blob([JSON.stringify(documentData,null,2)],{type:'application/json'}),url=URL.createObjectURL(blob),link=document.createElement('a');link.href=url;link.download='observaciones-calibracion.json';link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
  $('import').onchange=async()=>{try{const file=$('import').files[0];if(!file)return;const draft=JSON.parse(await file.text());if(!validDraft(draft))throw Error('El borrador no corresponde a estas imágenes/nubes o contiene pares inválidos.');applyDraft(draft);saveDraft();await setView(Number($('view').value));}catch(e){status(e.message);}finally{$('import').value='';}};
  interact(photoCanvas,true);interact(cloudCanvas,false);
  const resize=new ResizeObserver(()=>{for(const c of[photoCanvas,cloudCanvas]){c.width=Math.max(1,Math.round(c.clientWidth));c.height=Math.max(1,Math.round(c.clientHeight));}fitPhoto();drawCloud();});resize.observe(photoCanvas);resize.observe(cloudCanvas);
  await setView(0);
}
main().catch(e=>status(e.message));
