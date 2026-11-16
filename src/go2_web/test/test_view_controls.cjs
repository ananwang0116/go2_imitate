// Exercise the actual inline projection and pointer handlers, without a robot or a browser backend.
const fs=require('fs'), vm=require('vm'), assert=require('assert');
const html=fs.readFileSync(process.argv[2],'utf8');
const projection=html.slice(html.indexOf('function toS('),html.indexOf('function hue('));
const pointer=html.slice(html.indexOf('cv.onpointerdown='),html.indexOf('/* ---------- 定位 ---------- */'));
const context={assert,console,Math};
vm.createContext(context);
vm.runInContext(`
const S={view:'slice',cam:{cx:2,cy:-3,s:40,angle:0},stage:'build',wps:[],tool:null,follow:false,edits:[]};
const GL={ok:false,cam:{dist:30}},W=800,H=600;
const cv={getBoundingClientRect:()=>({left:0,top:0}),setPointerCapture:()=>{},style:{}};
const el={classList:{remove:()=>{}},value:.5};const $=()=>el;
const hitWp=()=>null,locked=()=>false,renderWps=()=>{},renderEdits=()=>{},renderGo=()=>{},renderRealtime=()=>{},hudTl=()=>{};
const endTool=()=>{S.tool=null};
const fx=()=>'',deg=()=>'';
let drag=null,toolMsg='',HINT={wall:''};
${projection}
${pointer}
const near=(a,b)=>assert(Math.abs(a-b)<1e-8, a+' != '+b);
const event=(x,y,button=0)=>({clientX:x,clientY:y,button,pointerId:1,preventDefault(){}});
for(const angle of [0,.7,Math.PI/2,Math.PI,-2.8]){
 S.cam.angle=angle;
 for(const p of [[0,0],[2,-3],[4.2,1.5],[-5,7]]){const s=toS(...p),w=toW(...s);near(w.x,p[0]);near(w.y,p[1])}
}
S.cam.angle=0;
cv.onpointerdown(event(400,300));cv.onpointermove(event(500,300));cv.onpointerup();near(S.cam.angle,.8);near(S.cam.cx,2);
const anchored=toW(300,200);cv.onpointerdown(event(300,200,2));cv.onpointermove(event(380,240,2));cv.onpointerup();
let w=toW(380,240);near(w.x,anchored.x);near(w.y,anchored.y);
const before=S.cam.s;cv.onpointerdown(event(400,300,1));cv.onpointermove(event(400,340,1));cv.onpointerup();assert(S.cam.s<before);
const wheelPoint=toW(210,230);cv.onwheel({...event(210,230),deltaY:-120});w=toW(210,230);near(w.x,wheelPoint.x);near(w.y,wheelPoint.y);
// Rotate the view, then place a pose and drag its WORLD heading to +y; no navigation/teleop request is made.
S.cam.angle=1.2;S.tool='pose';const start=toS(3,4),end=toS(3,5);
cv.onpointerdown(event(...start));cv.onpointermove(event(...end));cv.onpointerup();near(S.init.x,3);near(S.init.y,4);near(S.init.th,Math.PI/2);
// Right/middle gestures while a tool is active must not place a waypoint or wall.
S.tool='wall';cv.onpointerdown(event(100,100,2));cv.onpointermove(event(150,120,2));cv.onpointerup();assert.equal(S.edits.length,0);assert.equal(S.toolBuf,undefined);
cv.onpointercancel();assert.equal(drag,null);
console.log('PASS: rotated coordinate inverse, left rotation, right pan, middle drag/wheel zoom, pose heading, tool isolation');
`,context);

// Optional realtime heading survives rotated-view drag; clicking alone leaves it unspecified.
vm.runInContext(`
S.tool='rtPoint';const rs=toS(2,3),re=toS(2,4);
cv.onpointerdown(event(...rs));cv.onpointermove(event(...re));cv.onpointerup();near(S.rtTarget.yaw,Math.PI/2);
S.tool='rtPoint';cv.onpointerdown(event(...rs));cv.onpointerup();assert.equal(S.rtTarget.yaw,null);
`,context);
const smooth=html.slice(html.indexOf('function queueDisplayPose('),html.indexOf('function onState('));
vm.runInContext(`
let clock=0;const performance={now:()=>clock};const alignmentPending=st=>!!st.pending;
${smooth}
S.poseFrames=[];queueDisplayPose({pose:{x:0,y:0,th:3.1,frame:'map'}});
clock=100;queueDisplayPose({pose:{x:1,y:0,th:-3.1,frame:'map'}});
const mid=displayPose(200);near(mid.x,.5);assert(Math.abs(mid.th)>3);
near(displayPose(9999).x,1); // clamp: no extrapolation
queueDisplayPose({pose:null});assert.equal(displayPose(),null);
queueDisplayPose({pose:{x:0,y:0,th:0,frame:'map'}});
clock=200;queueDisplayPose({pose:{x:9,y:0,th:0,frame:'map'}});near(displayPose().x,9);
queueDisplayPose({pending:true,pose:{x:9,y:0,th:0,frame:'map'}});assert.equal(displayPose(),null);
`,context);
console.log('PASS realtime yaw, shortest-angle interpolation, no extrapolation, loss/reset');
