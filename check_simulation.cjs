// 檢查實際二維模擬中的能量、力矩與剛性約束，不需瀏覽器或額外套件。
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const file=require('node:path').join(__dirname,'index.html');
const html=fs.readFileSync(file,'utf8');
assert.ok(html.startsWith('<!doctype html>')&&html.includes('<html lang="zh-Hant">'),'交付必須是完整的繁體中文網頁');
assert.ok(!/<iframe\b|data-srcdoc=|window\.openai|<script[^>]+src=/i.test(html),'網頁須直接執行，不依賴內嵌 demo、ChatGPT 或外部程式');
const code=html.split('// BEGIN PHYSICS')[1].split('// END PHYSICS')[0];
const ctx=vm.createContext({});
vm.runInContext(code+'\nthis.sim={fields,designDefaults,defaultGeometry,buildGeometry,migrateDesign,along,rotate,hookBoardContact,forces,step,defaults,stickWidth,beamLength,crankPin,pinContacts,resolvePin,advanceMechanism,translateCrank,analyzeCrankCycle,whiteSweepPath,angleDelta,rotationBounds,limitRotation};',ctx);
const s=ctx.sim,near=(a,b,tol)=>assert.ok(Math.abs(a-b)<tol,`${a} ≠ ${b}`);
const screenshotDefaults={beamX:123.6,beamY:106,beamAngle:9.49,hookAngle:133.8,pivotT:54.1,beamT:38,join1:29,join2:60.8,angle2:-35.1};
for(const [key,value] of Object.entries(screenshotDefaults))near(s.designDefaults[key],value,1e-12);
const initialForce=s.forces(0,0);
near(initialForce.L,initialForce.path.slice(1).reduce((sum,p,i)=>sum+Math.hypot(...p.map((v,j)=>(v-initialForce.path[i][j])/1000)),0),1e-12);
assert.ok(s.defaultGeometry.binding.notch[1]>s.defaultGeometry.joint[1],'B 必須在設計姿勢的 J 上方');
// 直接解兩棍直邊的四個交點，確認選的是最高交點，沒有選到側邊或中心。
const normals=s.defaultGeometry.arms.map(arm=>{
  const dx=arm[1][0]-arm[0][0],dy=arm[1][1]-arm[0][1],L=Math.hypot(dx,dy);
  return [-dy/L,dx/L];
});
const [n1,n2]=normals,det=n1[0]*n2[1]-n1[1]*n2[0];
const edgeCrossings=[-5,5].flatMap(a=>[-5,5].map(b=>[
  s.defaultGeometry.joint[0]+(a*n2[1]-b*n1[1])/det,
  s.defaultGeometry.joint[1]+(n1[0]*b-n2[0]*a)/det
]));
const upperCrossing=edgeCrossings.reduce((best,p)=>p[1]>best[1]?p:best);
for(let i=0;i<2;i++)near(s.defaultGeometry.binding.notch[i],upperCrossing[i],1e-9);
assert.equal(initialForce.path.length,2,'橡筋只連接 A 接觸點及 B 凹位');
near(initialForce.L,Math.hypot(...s.defaultGeometry.binding.notch.map((v,i)=>(v-initialForce.a[i])/1000)),1e-12);
assert.ok(s.forces(0,0).spring<0&&s.forces(0,0).acceleration<0,'初始姿勢放手後須先順時針轉動');
near(s.forces(0,0,s.defaultGeometry,{...s.defaults,rest:0.2}).tension,0,1e-12);
const distance=(a,b)=>Math.hypot(a[0]-b[0],a[1]-b[1]);
const distanceToSegment=(p,a,b)=>{
  const dx=b[0]-a[0],dy=b[1]-a[1];
  const t=Math.max(0,Math.min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)));
  return distance(p,[a[0]+t*dx,a[1]+t*dy]);
};
// 用已知斜邊核對左側凹角，須取底板上邊，而非右側凹角、底板底邊或黏合中心。
const board=[[0,0],[30,0],[30,3],[0,3],[0,0]],slant=[[4,-2],[12,8],[16,8],[8,-2],[4,-2]];
near(distance(s.hookBoardContact(slant,board),[8,3]),0,1e-12);
const hookBefore=JSON.stringify(s.defaultGeometry.hook);
near(distance(s.forces(-Math.PI/2,0).a,s.forces(Math.PI/2,0).a),0,1e-12);
assert.equal(JSON.stringify(s.defaultGeometry.hook),hookBefore,'白棍轉動不能改變 A 棍位置');
assert.ok(distance(s.defaultGeometry.A,s.buildGeometry({...s.designDefaults,hookAngle:75}).A)>1,'調 A 棍角度時必須重新計算凹角位置');
function checkRubber(g,t){
  const f=s.forces(t,0,g);
  assert.ok(Number.isFinite(f.L)&&Number.isFinite(f.torque));
  if(!g.binding){assert.equal(f.valid,false);assert.equal(f.tension,0);return;}
  near(distance(f.b,s.rotate(g.binding.notch,t,g.P)),0,1e-9);
  assert.equal(f.valid,true);assert.equal(f.path.length,2);
  near(distance(f.path[0],f.a),0,1e-9);near(distance(f.path[1],f.b),0,1e-9);
  const edgeDistance=p=>Math.min(...g.hook.slice(1).map((end,i)=>distanceToSegment(p,g.hook[i],end)));
  near(edgeDistance(f.a),0,1e-9);
  near(distance(f.a,g.A),0,1e-9);
  near(f.a[1],g.base[2][1],1e-12);
  near(f.L,distance(f.a,f.b)/1000,1e-12);
  near(f.tension,Math.max(0,s.defaults.k*(distance(f.a,f.b)/1000-s.defaults.rest)),1e-12);
  for(const arm of g.arms){
    const L=distance(...arm),a=s.along(arm,s.stickWidth/2/L),b=s.along(arm,1-s.stickWidth/2/L);
    const contactDistance=distanceToSegment(g.binding.notch,a,b);
    assert.ok(contactDistance>=s.stickWidth/2-1e-6&&contactDistance<s.stickWidth/2+0.026,'凹位要同時接觸兩支棍的外沿');
  }
}
function checkAttachments(g){
  assert.deepEqual(Array.from(g.base[0]),[0,0],'底板必須固定在原 DXF 的位置');
  near(g.base[1][0]-g.base[0][0],210,1e-9);
  near(g.design.crankLength,21,1e-12);
  assert.ok(g.crankCenter[1]-g.base[2][1]>=21,'O 距綠色底板上邊至少 21mm');
  near(distance(s.crankPin(0,g),g.crankCenter),21,1e-9);
  near(distance(...g.beam),114,1e-9);
  for(const arm of g.arms)near(distance(...arm),114,1e-9);
  assert.equal(s.stickWidth,10,'三支長棍均用 114 × 10 mm');
  const start=s.along(g.beam,s.stickWidth/2/s.beamLength),end=s.along(g.beam,1-s.stickWidth/2/s.beamLength);
  near(distanceToSegment(g.contact,start,end),s.stickWidth/2,1e-8);
  near(g.support[2][1],g.contact[1],1e-9);
  near(g.support[0][1],g.design.baseY+3,1e-9);
  assert.ok(g.contact[0]>=g.support[0][0]-1e-9&&g.contact[0]<=g.support[1][0]+1e-9);
  assert.ok(g.support[0][0]>=g.design.baseX-1e-9&&g.support[1][0]<=g.design.baseX+210+1e-9);
  assert.ok(g.support[2][1]>=g.support[0][1]+5-1e-8);
  const foot=g.hook[0].map((v,i)=>(v+g.hook[g.hook.length-2][i])/2);
  near(distance(foot,g.glue),0,1e-9);
  assert.ok(g.glue[1]>=g.design.baseY&&g.glue[1]<=g.design.baseY+3);
  assert.ok(g.A,'A 棍與底板必須有凹角交界');
  near(g.A[1],g.base[2][1],1e-12);
  near(Math.min(...g.hook.slice(1).map((end,i)=>distanceToSegment(g.A,g.hook[i],end))),0,1e-9);
  assert.ok(g.A[0]>=g.base[0][0]&&g.A[0]<=g.base[1][0]);
}
const variants=[s.defaultGeometry];
for(const [key,label,min,max] of s.fields){
  variants.push(s.buildGeometry({...s.designDefaults,[key]:min}),s.buildGeometry({...s.designDefaults,[key]:max}));
}
for(const g of variants){
  checkAttachments(g);
  near(distance(s.along(g.arms[0],g.design.join1/100),s.along(g.arms[1],g.design.join2/100)),0,1e-9);
  assert.ok(g.inertiaPerMass>0);
for(const t of [-Math.PI,-1,0,1,Math.PI]){
  checkRubber(g,t);
  for(const arm of g.arms){
    const a=s.rotate(arm[0],t,g.P),b=s.rotate(arm[1],t,g.P);
    near(Math.hypot(b[0]-a[0],b[1]-a[1]),Math.hypot(arm[1][0]-arm[0][0],arm[1][1]-arm[0][1]),1e-9);
  }
  near(distance(s.rotate(g.arms[0][0],t,g.P),s.rotate(g.arms[1][0],t,g.P)),distance(g.arms[0][0],g.arms[1][0]),1e-9);
  const h=1e-6,params={...s.defaults,damping:0};
  near(s.forces(t,0,g,params).torque,-(s.forces(t+h,0,g,params).energy-s.forces(t-h,0,g,params).energy)/(2*h),1e-8);
}
}
for(let hookAngle=15;hookAngle<=165;hookAngle+=5)checkAttachments(s.buildGeometry({...s.designDefaults,hookAngle}));
for(let beamAngle=-180;beamAngle<=180;beamAngle+=5)for(const beamX of [-100,450])for(const beamY of [0,300]){
  checkAttachments(s.buildGeometry({...s.designDefaults,beamAngle,beamX,beamY,baseX:beamX<0?-100:150,baseY:beamY===0?100:-60}));
}
const migrated=s.buildGeometry(s.migrateDesign({px:86.67402113077394,py:96.91483985701933},3));
near(migrated.P[0],86.67402113077394,1e-9);near(migrated.P[1],96.91483985701933,1e-9);
let notchSweep=0;
for(let angle2=-150;angle2<=150;angle2+=10)for(const join1 of [5,50,95])for(const join2 of [5,50,95]){
  checkRubber(s.buildGeometry({...s.designDefaults,angle2,join1,join2}),0);notchSweep++;
}
const g0=s.defaultGeometry,zero={...g0,A:[...g0.binding.notch]};
const unrestricted=g=>({...g,design:{...g.design,rotationMin:-180,rotationMax:180}}),cycleGeometry=unrestricted(g0);
assert.equal(g0.design.rotationMin,-50);assert.equal(g0.design.rotationMax,50);
const focusedBounds=s.rotationBounds(g0);near(focusedBounds[0],-50*Math.PI/180,1e-12);near(focusedBounds[1],50*Math.PI/180,1e-12);
for(const sign of [-1,1]){const limited=s.limitRotation([sign*Math.PI,sign*2],g0);near(limited[0],sign*50*Math.PI/180,1e-12);assert.equal(limited[1],0);}
const oldCrank=s.buildGeometry({...s.designDefaults,crankLength:60,crankY:-100,crankRPM:-45});
near(oldCrank.design.crankLength,21,1e-12);near(oldCrank.crankCenter[1]-oldCrank.base[2][1],21,1e-12);assert.equal(oldCrank.design.crankRPM,45);
const oldBinding=s.buildGeometry(s.migrateDesign({...s.designDefaults,bandArm:2,bandT:95,bandSide:4},5));
near(distance(s.forces(0,0,oldBinding).b,oldBinding.binding.notch),0,1e-9);
assert.ok(!s.fields.some(([key])=>key.startsWith('band')),'橡筋端必須卡在外沿凹位，不能另行移位');
assert.equal(s.forces(0,0,s.buildGeometry({...s.designDefaults,angle2:0})).valid,false,'兩棍重疊沒有凹位，不能假裝綁在中心');
checkAttachments(s.buildGeometry({...s.designDefaults,baseX:150,baseY:100}));
assert.ok(!s.fields.some(([key])=>key==='baseX'||key==='baseY'),'不能再獨立移動底板');
near(s.buildGeometry(null).P[0],g0.P[0],1e-12);
assert.ok(Number.isFinite(s.forces(0,0,zero).acceleration),'A 接觸點與 B 重合不能令計算變成 NaN');
near(s.forces(0,0,zero).tension,0,1e-12);
assert.ok(Math.abs(s.buildGeometry({...s.designDefaults,pivotT:10}).inertiaPerMass-g0.inertiaPerMass)>1e-5,'樞軸調位後要重算轉動慣量');
let q=[0,0],params={...s.defaults,damping:0};
const energy=s.forces(...q,g0,params).energy;
let energyDrift=0;
// A 留在固定凹角，守恆檢查用半步長降低 RK4 誤差，總時間仍為 10 秒。
for(let i=0;i<20000;i++){
  q=s.step(q,0.0005,g0,params);
  energyDrift=Math.max(energyDrift,Math.abs(s.forces(...q,g0,params).energy-energy));
  near(s.forces(...q,g0,params).energy,energy,1e-8);
}
q=[0,0];let previous=s.forces(...q).energy;
for(let i=0;i<12000;i++){
  q=s.step(q,0.001);
  const next=s.forces(...q).energy;
  assert.ok(next<=previous+1e-10,'有阻尼時機械能不能增加');previous=next;
}
assert.ok(Math.abs(q[1])<0.001&&Math.abs(s.forces(...q).acceleration)<0.01,'需停在受力平衡附近');
// 圓銷依實際圓頭棍外沿作單向接觸；完整循環用明確無擋位的物理測試配置。
for(const phase of [0,Math.PI/2,Math.PI,Math.PI*3/2])near(distance(s.crankPin(phase),g0.crankCenter),s.designDefaults.crankLength,1e-9);
const upper=s.along(g0.arms[0],0.85),unit=g0.arms[0][1].map((v,i)=>(v-g0.arms[0][0][i])/114),normal=[-unit[1],unit[0]];
const contactPin=upper.map((v,i)=>v+normal[i]*8),contactGeometry={...g0,crankCenter:[contactPin[0]-5,contactPin[1]],design:{...g0.design,crankLength:5,pinRadius:3}};
const edge=s.pinContacts(0,0,contactGeometry)[0];near(edge.gap,0,1e-9);
const pushed=s.resolvePin([0,0],0,1,contactGeometry);assert.equal(pushed.jammed,false);near(pushed.state[1],normal[1]*5/edge.lever,1e-9);
near(s.resolvePin([0,0],0,-1,contactGeometry).state[1],0,1e-12,'圓銷離開時不能拉住白棍');
// O 平移亦須經過接觸路徑，推動白棍；離開及受阻時保留最後可行位置。
const slideStart=upper.map((v,i)=>v+normal[i]*12-(i?21:0));
const slideGeometry=s.buildGeometry({...s.designDefaults,crankX:slideStart[0],crankY:slideStart[1]});
const slideTarget=slideStart.map((v,i)=>v-normal[i]*16);
const shifted=s.translateCrank([0,0],Math.PI/2,slideTarget,slideGeometry);
assert.equal(shifted.jammed,false);assert.ok(Math.abs(shifted.state[0])>Math.PI/180);
near(distance(shifted.geometry.crankCenter,slideTarget),0,1e-9);
for(const c of s.pinContacts(shifted.state[0],Math.PI/2,shifted.geometry))assert.ok(c.gap>=-.001);
near(distance(shifted.geometry.P,slideGeometry.P),0,1e-12);assert.deepEqual(shifted.geometry.arms,slideGeometry.arms);
const withdrawn=s.translateCrank(shifted.state,Math.PI/2,slideStart,shifted.geometry);
assert.equal(withdrawn.jammed,false);near(withdrawn.state[0],shifted.state[0],1e-8,'移走 O 不能拉住白棍');
const swept=s.translateCrank([0,0],Math.PI/2,slideStart.map((v,i)=>v-normal[i]*32),slideGeometry);
assert.equal(swept.jammed,false);assert.ok(Math.abs(swept.state[0])>Math.PI/180,'大幅移動 O 亦不能跳過白棍');
for(const c of s.pinContacts(swept.state[0],Math.PI/2,swept.geometry))assert.ok(c.gap>=-.001);
const stoppedShift=s.translateCrank([0,0],Math.PI/2,slideTarget,s.buildGeometry({...slideGeometry.design,rotationMin:-2,rotationMax:2}));
assert.equal(stoppedShift.jammed,true);assert.equal(stoppedShift.reason,'limit');
assert.ok(stoppedShift.state[0]>=-2*Math.PI/180&&stoppedShift.state[0]<=2*Math.PI/180);
assert.ok(distance(stoppedShift.geometry.crankCenter,slideTarget)>1,'到 P 擋位時 O 亦須停下');
for(const c of s.pinContacts(stoppedShift.state[0],Math.PI/2,stoppedShift.geometry))assert.ok(c.gap>=-.001);
const clearGeometry={...g0,crankCenter:[200,170],design:{...g0.design,crankLength:5}};
const freeState=s.advanceMechanism([0,0],0,0.001,clearGeometry,0);assert.equal(freeState.touching,false);
for(let i=0;i<2;i++)near(freeState.state[i],s.step([0,0],0.001,clearGeometry)[i],1e-12);
const blocked={...g0,crankCenter:[g0.P[0]-5,g0.P[1]],design:{...g0.design,crankLength:5}};
assert.equal(s.resolvePin([0,0],0,1,blocked).jammed,true,'圓銷穿到 P 時不可假裝白棍能避開');
let crankState=[0,0],phase=s.designDefaults.crankAngle*Math.PI/180,contactSteps=0,clearSteps=0,minAngle=0,maxAngle=0;
const crankCycles=Array.from({length:3},()=>({contact:0,clear:0,rubberReturn:0}));
const omega=-s.designDefaults.crankRPM*Math.PI/30;
for(let i=0;i<6000;i++){
  const result=s.advanceMechanism(crankState,phase,0.001,cycleGeometry,omega);
  assert.equal(result.jammed,false,`預設曲柄不能卡死：step=${i}, angle=${crankState[0]}, phase=${phase}`);
  crankState=result.state;phase=result.phase;minAngle=Math.min(minAngle,crankState[0]);maxAngle=Math.max(maxAngle,crankState[0]);
  result.touching?contactSteps++:clearSteps++;
  const cycle=crankCycles[Math.floor(i/2000)];result.touching?cycle.contact++:cycle.clear++;
  if(!result.touching&&crankState[1]<-0.01&&s.forces(...crankState,cycleGeometry).spring<0)cycle.rubberReturn++;
  for(const c of s.pinContacts(crankState[0],phase,cycleGeometry))assert.ok(c.gap>=-0.001,'圓銷不能穿過任一白棍外沿');
  assert.ok(crankState.every(Number.isFinite));
}
near(phase,s.designDefaults.crankAngle*Math.PI/180+6*omega,1e-9);
assert.ok(contactSteps>100&&clearSteps>0,`預設曲柄須有推棍及初始分離階段：${JSON.stringify({contactSteps,clearSteps,minAngle,maxAngle})}`);
assert.ok(crankCycles.every(c=>c.contact>0),`預設曲柄每圈都須推棍：${JSON.stringify(crankCycles)}`);
assert.ok(maxAngle-minAngle>Math.PI/9,'曲柄應推動白棍繞 P 轉動超過 20 度');
// 移動 O 後，21mm 順時針曲柄仍須能在每圈分離，讓橡筋拉回。
const returnGeometry=unrestricted(s.buildGeometry({...s.designDefaults,crankX:70,crankY:80}));
let returnState=[0,0],returnPhase=Math.PI/2;
const returnCycles=Array.from({length:3},()=>({contact:0,clear:0,rubberReturn:0}));
for(let i=0;i<6000;i++){
  const result=s.advanceMechanism(returnState,returnPhase,.001,returnGeometry,omega);
  assert.equal(result.jammed,false);returnState=result.state;returnPhase=result.phase;
  const cycle=returnCycles[Math.floor(i/2000)];result.touching?cycle.contact++:cycle.clear++;
  if(!result.touching&&returnState[1]<-.01&&s.forces(...returnState,returnGeometry).spring<0)cycle.rubberReturn++;
  for(const c of s.pinContacts(returnState[0],returnPhase,returnGeometry))assert.ok(c.gap>=-.001);
}
assert.ok(returnCycles.every(c=>c.contact>0&&c.clear>0&&c.rubberReturn>0),'拖動 O 後，圓銷離開時橡筋須可拉回白棍');
near(s.angleDelta(-179*Math.PI/180,179*Math.PI/180),2*Math.PI/180,1e-12);
near(s.angleDelta(179*Math.PI/180,-179*Math.PI/180),-2*Math.PI/180,1e-12);
const limitedGeometry={...g0,design:{...g0.design,rotationMin:-10,rotationMax:10}};
const [lowerStop,upperStop]=s.rotationBounds(limitedGeometry);
near(s.limitRotation([lowerStop-1,-10],limitedGeometry)[0],lowerStop,1e-12);assert.equal(s.limitRotation([lowerStop-1,-10],limitedGeometry)[1],0);
near(s.limitRotation([upperStop+1,10],limitedGeometry)[0],upperStop,1e-12);assert.equal(s.limitRotation([upperStop+1,10],limitedGeometry)[1],0);
near(s.limitRotation([Math.PI*8,2],cycleGeometry)[0],Math.PI*8,1e-12,'完整 360 度不限制連續轉動');
let limitedState=[0,0],limitedPhase=Math.PI/2,limitStopped=false;
for(let i=0;i<3000;i++){
  const result=s.advanceMechanism(limitedState,limitedPhase,0.001,limitedGeometry,omega);
  limitedState=result.state;limitedPhase=result.phase;
  assert.ok(limitedState[0]>=lowerStop-1e-10&&limitedState[0]<=upperStop+1e-10,'白棍不能越過任何角度邊界');
  for(const c of s.pinContacts(...[limitedState[0],limitedPhase,limitedGeometry]))assert.ok(c.gap>=-0.001,'到達轉角限制也不能讓圓銷穿棍');
  if(result.jammed){assert.equal(result.reason,'limit');limitStopped=true;break;}
}
assert.equal(limitStopped,true,'曲柄撞上被轉角限制擋住的白棍時必須停止');
// 一圈的接觸／橡筋回拉分段要來自同一套物理，不能預設各佔半圈。
const analysisState=[0,0],analysisGeometry=JSON.stringify(g0);
const cycleReport=s.analyzeCrankCycle(analysisState,Math.PI/2,g0);
assert.equal(cycleReport.complete,true);near(cycleReport.degrees,360,1e-8);
assert.deepEqual(analysisState,[0,0]);assert.equal(JSON.stringify(g0),analysisGeometry,'分析不能移動實際機構');
assert.equal(cycleReport.segments[0].kind,'return');near(cycleReport.segments[0].from,0,1e-12);
near(cycleReport.segments[0].to,8.3,.2);
assert.ok(cycleReport.segments.some(s=>s.kind==='contact'&&s.to-s.from>340));
let end=0;
for(const segment of cycleReport.segments){near(segment.from,end,1e-8);assert.ok(segment.to>segment.from);end=segment.to;}
near(end,360,1e-8);
const pushRange=cycleReport.segments.find(s=>s.kind==='contact'),rubberRange=cycleReport.segments.find(s=>s.kind==='return');
near(rubberRange.fromAngle,0,1e-12);assert.ok(rubberRange.toAngle<0);near(rubberRange.maxAngle,0,1e-12);
assert.ok(pushRange.minAngle<-30&&pushRange.maxAngle>10,'要量白棍的擺角，不能拿曲柄的 360° 當白棍角度');
for(const segment of cycleReport.segments){
  assert.ok(segment.minAngle>=-50&&segment.maxAngle<=50);
  assert.ok(segment.minAngle<=segment.fromAngle&&segment.fromAngle<=segment.maxAngle);
  assert.ok(segment.minAngle<=segment.toAngle&&segment.toAngle<=segment.maxAngle);
}
// 掃過範圍保留兩支 114 × 10 mm 圓頭棍的形狀，取樣包括旋轉的兩個邊界。
const sweepPath=s.whiteSweepPath([[-30,20]],g0),sweepLines=[...sweepPath.matchAll(/M([^ ]+?)L([^ ]+)(?: |$)/g)];
assert.equal(sweepLines.length,202);
for(const [line,arm,angle] of [[sweepLines[0],g0.arms[0],-30],[sweepLines.at(-1),g0.arms[1],20]]){
  const ends=[line[1],line[2]].map(p=>p.split(',').map(Number));
  for(const [i,fraction] of [[0,5/114],[1,109/114]])near(distance(ends[i],s.rotate(s.along(arm,fraction),angle*Math.PI/180,g0.P)),0,.001);
  near(distance(...ends)+10,114,.002);
}
assert.equal(s.whiteSweepPath([[-30,0],[-10,20]],g0),sweepPath,'重疊角度範圍要合併，不可重複畫棍');
assert.equal(s.whiteSweepPath([],g0),'');

const detachedReport=s.analyzeCrankCycle([0,0],Math.PI/2,s.buildGeometry({...s.designDefaults,crankX:190,crankY:170}));
assert.equal(detachedReport.complete,true);assert.ok(!detachedReport.segments.some(s=>s.kind==='contact'));
assert.ok(detachedReport.segments.some(s=>s.kind==='return')&&detachedReport.segments.some(s=>s.kind==='free'),'Q 離開時要分清橡筋收縮與其他狀態');
const blockedReport=s.analyzeCrankCycle([0,0],Math.PI/2,limitedGeometry);
assert.equal(blockedReport.complete,false);assert.equal(blockedReport.reason,'limit');
assert.ok(blockedReport.degrees>0&&blockedReport.degrees<360);
near(blockedReport.segments.reduce((sum,s)=>sum+s.to-s.from,0),blockedReport.degrees,1e-8,'受阻後不能填入虛構的剩餘角度');
const stationaryReport=s.analyzeCrankCycle([0,0],Math.PI/2,{...g0,design:{...g0.design,crankRPM:0}});
assert.equal(stationaryReport.reason,'speed');assert.equal(stationaryReport.segments.length,0);

// 白棍以向外的速度接近新預設擋位，時間積分亦須停在 ±50°。
for(const sign of [-1,1]){
  const result=s.advanceMechanism([sign*49.99*Math.PI/180,sign*5],Math.PI/2,.001,clearGeometry,0);
  near(result.state[0],sign*50*Math.PI/180,1e-12);assert.equal(result.state[1],0);
  for(const c of s.pinContacts(result.state[0],result.phase,clearGeometry))assert.ok(c.gap>=-.001);
}
assert.ok(!html.includes('\\"'),'HTML 必須是原生標記');
// 驗證同一段互動程式會綁定所有控件，並正確處理舊設定與放手／還原；這不是瀏覽器版面驗收。
const elements=new Map(),globals=new Map(),frames=new Map();let frameId=0,savedState=null;
const guideTimers=new Map();let guideTimerId=0;
let viewportWidth=736,canvasHeight=0,viewportLeft=0,viewportTop=0,inResizeNotification=false,resizeCallback,resizeTarget;
class Element {
  constructor(id=''){this.id=id;this.value='';this.style={};this.events=new Map();this.content='';this.min=0;this.max=0;this.disabled=false;this.open=false;}
  set innerHTML(value){
    if(this.id==='mechanism')assert.ok(!inResizeNotification,'尺寸通知派發期間不能重繪同一個 SVG');
    this.content=value;
    const match=value.match(/<input[^>]*id="([^"]+)"[^>]*min="([^"]+)"[^>]*max="([^"]+)"/);
    if(match){this.slider=new Element(match[1]);this.slider.min=Number(match[2]);this.slider.max=Number(match[3]);elements.set(match[1],this.slider);this.output=new Element();}
  }
  get innerHTML(){return this.content;}
  querySelector(selector){if(selector==='input')return this.slider;if(selector==='output')return this.output;return elements.get(selector.replace('#',''));}
  addEventListener(name,callback){this.events.set(name,callback);}
  appendChild(){}
  focus(options){this.focused=true;this.focusOptions=options;}
  scrollIntoView(options){this.scrollOptions=options;}
  setAttribute(name,value){this[name]=value;}
  getBoundingClientRect(){return {width:viewportWidth,height:canvasHeight,left:this.id==='mechanism'?viewportLeft:0,top:this.id==='mechanism'?viewportTop:0};}
  setPointerCapture(id){this.capturedPointer=id;}
  hasPointerCapture(id){return this.capturedPointer===id;}
  releasePointerCapture(){this.capturedPointer=undefined;}
}
for(const [,id] of html.matchAll(/\bid="([^"]+)"/g))elements.set(id,new Element(id));
elements.set('.mechanism',new Element('mechanism'));
const location={pathname:'/index.html',search:''};
const storageKey=`codex:visualization-widget-state-v2:${JSON.stringify([location.pathname,location.search])}`;
const initialStorageKey=`${storageKey}:initial`;
const initialSaved={modelContent:{version:3,angle:60.7,design:{px:86.67402113077394,py:96.91483985701933,bandArm:2,bandT:95,bandSide:4}},privateContent:{control:'bandSide'}};
const storageValues=new Map([[storageKey,JSON.stringify(initialSaved)]]);
const localStorage={
  getItem:key=>{assert.ok([storageKey,initialStorageKey].includes(key));return storageValues.get(key)??null;},
  setItem:(key,value)=>{assert.ok([storageKey,initialStorageKey].includes(key));storageValues.set(key,value);if(key===storageKey)savedState=JSON.parse(value);}
};
const runtime=vm.createContext({
  document:{getElementById:id=>elements.get(id),createElement:()=>new Element()},
  window:{location,innerWidth:1600,innerHeight:900,matchMedia:()=>({get matches(){return runtime.window.innerWidth>=600&&runtime.window.innerWidth<=1366;}}),addEventListener:(name,callback)=>globals.set(name,callback)},localStorage,
  ResizeObserver:class {constructor(callback){resizeCallback=callback;}observe(target){resizeTarget=target;}},requestAnimationFrame:callback=>{frames.set(++frameId,callback);return frameId;},cancelAnimationFrame:id=>frames.delete(id),
  setTimeout:(callback,delay)=>{guideTimers.set(++guideTimerId,{callback,delay});return guideTimerId;},clearTimeout:id=>guideTimers.delete(id)
});
vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1],runtime);
function storageEvent(value){globals.get('storage')({key:storageKey,newValue:JSON.stringify(value)});}
function checkRubberSvg(){
  const content=elements.get('.mechanism').content;
  const band=content.match(/<path d="([^"]+)" fill="none" stroke="var\(--orange\)" stroke-width="2"/);
  assert.ok(band,'圖中需有 A–B 橡筋');
  const ends=band[1].match(/^M([^ ]+) L([^ ]+)$/);assert.ok(ends,'橡筋需是一條直線，不能沿棍身繞行');
  const a=ends[1].split(',').map(Number),b=ends[2].split(',').map(Number);
  const fixed=content.match(/<g data-rubber-contact><circle cx="([^"]+)" cy="([^"]+)" r="3.5" fill="var\(--green\)"/);
  const moving=content.match(/<circle cx="([^"]+)" cy="([^"]+)" r="3.5" fill="var\(--orange\)"/);
  assert.ok(fixed&&moving);
  near(a[0],Number(fixed[1]),1e-9);near(a[1],Number(fixed[2]),1e-9);
  near(b[0],Number(moving[1]),1e-9);near(b[1],Number(moving[2]),1e-9);
  const base=content.match(/<path d="([^"]+)" fill="none" stroke="var\(--green\)"/)[1].split(' ').map(p=>p.slice(1).split(',').map(Number));
  near(a[1],base[2][1],1e-9);
  assert.ok(a[0]>=base[0][0]&&a[0]<=base[1][0],'SVG 的 A 點必須落在底板上邊');
}
checkRubberSvg();
assert.ok(elements.get('rubber-sim-cycle-segments').content.includes('data-phase="contact"'));
assert.ok(elements.get('rubber-sim-cycle-segments').content.includes('data-phase="return"'));
assert.ok(elements.get('rubber-sim-cycle-note').textContent.includes('30.0 rpm'));
assert.ok(elements.get('rubber-sim-cycle-ranges').content.includes('360.0°'));
assert.ok(elements.get('rubber-sim-cycle-ranges').content.includes('白棍')&&elements.get('rubber-sim-cycle-ranges').content.includes('起點'));
assert.ok(elements.get('.mechanism').content.includes('data-white-swept-area="contact"')&&elements.get('.mechanism').content.includes('data-white-swept-area="return"'));
assert.ok(elements.get('.mechanism').content.includes('data-white-angle-range='));
assert.equal(elements.get('rubber-sim-cycle-marker').style.left,'50%','初始白棍 0° 在 −50° 至 50° 中間');

// 模擬首次尺寸通知、寬度改變、重繪後高度通知，檢查不會循環重繪。
function notifyResize(width,height){
  viewportWidth=width;canvasHeight=height;inResizeNotification=true;
  try{resizeCallback([{contentRect:{width,height}}]);}finally{inResizeNotification=false;}
}
assert.equal(resizeTarget.id,'rubber-sim-canvas','應觀察獨立畫布格，避免 SVG 寫入自身高度時觸發循環');
notifyResize(736,300);assert.equal(frames.size,0);
notifyResize(400,300);assert.equal(frames.size,1);
const [resizeFrame,resizeDraw]=frames.entries().next().value;frames.delete(resizeFrame);resizeDraw();
assert.ok(elements.get('.mechanism').viewBox.startsWith('0 0 400 '));
notifyResize(400,424);assert.equal(frames.size,0,'只改高度不能再次安排重繪');
assert.equal(elements.get('rubber-sim-angle-value').value,'0.0°','舊快照不能覆蓋新的初始姿勢');
for(const [key,value] of Object.entries(screenshotDefaults))near(Number(elements.get('rubber-sim-'+key).value),value,1e-12);
assert.equal(elements.get('rubber-sim-select-pivot')['aria-pressed'],'true','開啟網頁時集中顯示 P 角度範圍');
assert.equal(elements.get('rubber-sim-pivot-controls').hidden,false);
assert.equal(elements.get('rubber-sim-joint-controls').hidden,true);
for(const key of ['rotationMin','rotationMax','angle']){const slider=elements.get('rubber-sim-'+key);assert.equal(Number(slider.min),-50);assert.equal(Number(slider.max),50);}
assert.ok(!elements.get('.mechanism').content.includes('data-control='),'載入時不能長期顯示藍色提示');
assert.ok(!elements.get('.mechanism').content.includes('data-crank-orbit'),'閒置時隱藏曲柄輔助圓周');
for(const key of ['bandArm','bandT','bandSide'])assert.ok(!elements.has('rubber-sim-'+key));
const visibleFields=s.fields.filter(([key])=>!['pinRadius','crankRPM','crankAngle','crankLength'].includes(key));
for(const [key] of visibleFields){
  const slider=elements.get('rubber-sim-'+key);assert.ok(slider?.events.has('input')&&slider.events.has('change'));
  assert.ok(slider.events.has('focus')&&slider.events.has('pointerenter'),'每條 slider 均需要圖形提示');
  slider.events.get('focus')();
  checkRubberSvg();
  assert.ok(elements.get('.mechanism').content.includes(`data-control="${key}"`),'圖中提示必須跟隨正在調整的 slider');
  assert.ok(!elements.get('.mechanism').content.includes('NaN'));
  if(['join1','join2','angle2'].includes(key)){
    const content=elements.get('.mechanism').content;
    const anchor=content.match(/<circle cx="([^"]+)" cy="([^"]+)" r="3.5" fill="var\(--orange\)"/);
    const mark=content.split(`<g data-control="${key}">`)[1].match(/<circle cx="([^"]+)" cy="([^"]+)" r="10"/);
    assert.ok(anchor&&mark);assert.ok(Math.hypot(Number(anchor[1])-Number(mark[1]),Number(anchor[2])-Number(mark[2]))>1,'凹位 B 要與 J 中心分開');
    assert.equal((content.match(/>J<\/text>/g)||[]).length,1);
    assert.equal((content.match(/>B<\/text>/g)||[]).length,1,'凹位 B 與接合點 J 均需標記');
  }
}
assert.equal(guideTimers.size,1,'只有最近一次調整可以安排收起提示');
const selectedPin=elements.get('.mechanism').content.match(/<circle data-crank-pin[^>]+\/>/)[0];
const [guideTimer,{callback:hideGuide,delay:guideDelay}]=guideTimers.entries().next().value;
assert.equal(guideDelay,800);guideTimers.delete(guideTimer);hideGuide();
assert.ok(!elements.get('.mechanism').content.includes('data-control='),'停止調整後須自動收起提示');
assert.ok(!elements.get('.mechanism').content.includes('data-crank-orbit'));
assert.equal(elements.get('.mechanism').content.match(/<circle data-crank-pin[^>]+\/>/)[0],selectedPin,'收起提示不能移動曲柄');
assert.equal(elements.get('rubber-arm-simulation')['data-guides'],'false','控制旁的藍點亦須收起');
assert.ok(elements.get('.mechanism').content.includes('data-context="fixed" opacity="0.5"'));
assert.ok(elements.get('.mechanism').content.includes('data-context="rubber" opacity="0.55"'));
assert.ok(!elements.get('.mechanism').content.includes('rubber-sim-force-arrow'),'移除持續顯示的拉力箭嘴');
assert.ok(!elements.has('rubber-sim-baseX')&&!elements.has('rubber-sim-baseY'));
elements.get('rubber-sim-select-hook').events.get('click')();
assert.equal(elements.get('rubber-sim-hook-controls').hidden,false);
assert.equal(elements.get('rubber-sim-beam-controls').hidden,true);
assert.equal(elements.get('rubber-sim-select-hook')['aria-pressed'],'true');
for(const [key,coordinate] of [['beamX',1],['beamY',0]]){
  elements.get('rubber-sim-'+key).events.get('focus')();
  const guide=elements.get('.mechanism').content.split(`<g data-control="${key}">`)[1];
  const match=guide.match(/<path d="M([\d.,-]+) L([\d.,-]+)"[^>]*marker-start=/);
  assert.ok(match,'移位提示需要方向箭嘴');
  const a=match[1].split(',').map(Number),b=match[2].split(',').map(Number);
  near(a[coordinate],b[coordinate],1e-9);assert.ok(Math.abs(a[1-coordinate]-b[1-coordinate])>1);
}
elements.get('rubber-sim-beamAngle').events.get('focus')();
assert.ok(elements.get('.mechanism').content.split('<g data-control="beamAngle">')[1].includes(' A'),'角度提示需要轉動弧線');
const hookSlider=elements.get('rubber-sim-hookAngle');hookSlider.value='75';hookSlider.events.get('input')();hookSlider.events.get('change')();
assert.equal(savedState.modelContent.version,13);assert.equal(savedState.modelContent.design.hookAngle,75);
assert.equal(elements.get('rubber-sim-cycle-segments').content,'','改動配置後不能保留過時角度分段');
assert.ok(elements.get('rubber-sim-cycle-note').textContent.includes('改動'));
assert.ok(!elements.get('.mechanism').content.includes('data-white-swept-area='),'移動部件後要清走舊掃過範圍');
checkRubberSvg();
assert.equal(savedState.privateContent.control,'hookAngle');
assert.equal(savedState.modelContent.design.baseX,0);assert.equal(savedState.modelContent.design.baseY,0);
const heightSlider=elements.get('rubber-sim-beamY');heightSlider.value='0';heightSlider.events.get('input')();heightSlider.events.get('change')();
assert.ok(savedState.modelContent.design.beamY>0&&heightSlider.min>0);
heightSlider.value=String(s.designDefaults.beamY);heightSlider.events.get('input')();
const angle2Slider=elements.get('rubber-sim-angle2');angle2Slider.value='0';angle2Slider.events.get('input')();
assert.equal(elements.get('rubber-sim-play').disabled,true);assert.ok(elements.get('rubber-sim-status').textContent.includes('沒有凹位'));
assert.equal(elements.get('rubber-sim-analyze').disabled,false,'配置不能轉動時仍須容許分析並解釋原因');
elements.get('rubber-sim-analyze').events.get('click')();
assert.ok(elements.get('rubber-sim-status').textContent.includes('兩棍沒有凹位'));
assert.equal(elements.get('rubber-sim-cycle-panel').open,true);
angle2Slider.value=String(s.designDefaults.angle2);angle2Slider.events.get('input')();
assert.equal(elements.get('rubber-sim-play').disabled,false);
const play=elements.get('rubber-sim-play');play.events.get('click')();
for(const timestamp of [0,16,32]){const [id,callback]=frames.entries().next().value;frames.delete(id);callback(timestamp);}
assert.notEqual(elements.get('rubber-sim-angle-value').value,'0.0°');
play.events.get('click')();assert.equal(play.textContent,'繼續');
elements.get('rubber-sim-home').events.get('click')();assert.equal(elements.get('rubber-sim-angle-value').value,'0.0°');
for(const [key,value] of Object.entries(screenshotDefaults))near(Number(elements.get('rubber-sim-'+key).value),value,1e-12);
storageEvent({modelContent:{version:9,angle:60.7,design:{...s.designDefaults,beamX:150,hookAngle:75}}});
for(const [key,value] of Object.entries(screenshotDefaults))near(Number(elements.get('rubber-sim-'+key).value),value,1e-12);
assert.equal(elements.get('rubber-sim-angle-value').value,'0.0°','延遲收到的舊快照亦不能覆蓋初始配置');
const workingStorage=runtime.localStorage.setItem;runtime.localStorage.setItem=()=>{throw new Error('儲存被拒絕');};
assert.doesNotThrow(()=>elements.get('rubber-sim-home').events.get('click')(),'本機儲存不可用時模擬仍須正常');
runtime.localStorage.setItem=workingStorage;
assert.doesNotThrow(()=>globals.get('storage')({key:storageKey,newValue:'錯誤 JSON'}),'損壞的保存資料不能令頁面失效');
function canvasDrawing(){
  const svg=elements.get('.mechanism');
  return {viewBox:svg.viewBox,height:svg.style.height,green:[...svg.content.matchAll(/<path\b[^>]*stroke="var\(--green\)"[^>]*\/>/g)].map(match=>match[0])};
}
function movingDrawing(){
  return [...elements.get('.mechanism').content.matchAll(/<path d="([^"]+)" stroke="var\(--foreground\)" stroke-width="([^"]+)" stroke-linecap="round"\/>/g)];
}
const contactCanvas=canvasDrawing(),contactAngle=elements.get('rubber-sim-angle');
const contactDrawing=()=>elements.get('.mechanism').content.match(/<g data-rubber-contact><circle[^>]+\/>/)[0];
contactAngle.value='-50';contactAngle.events.get('input')();checkRubberSvg();const lowerContact=contactDrawing();
contactAngle.value='50';contactAngle.events.get('input')();checkRubberSvg();
assert.equal(contactDrawing(),lowerContact,'白棍轉動時 SVG 的 A 點必須保持卡在底板凹角');
assert.deepEqual(canvasDrawing(),contactCanvas,'白棍轉動不能移動或縮放固定件');
elements.get('rubber-sim-home').events.get('click')();
const fixedBefore=canvasDrawing(),movingBefore=movingDrawing().map(match=>match[0]);
const drive=elements.get('rubber-sim-drive'),pinDrawing=()=>elements.get('.mechanism').content.match(/<circle data-crank-pin[^>]+\/>/)[0];
const pinBefore=pinDrawing();drive.events.get('click')();
assert.ok(!elements.get('.mechanism').content.includes('data-control='),'運轉時不選出藍色線');
assert.equal(guideTimers.size,0,'啟動曲柄時取消待收起的提示');
elements.get('rubber-sim-crankY').events.get('pointerenter')();
assert.ok(!elements.get('.mechanism').content.includes('data-control='),'運轉時掠過 slider 亦不能覆蓋藍線');
for(const timestamp of [0,16,32,48]){const [id,callback]=frames.entries().next().value;frames.delete(id);callback(timestamp);}
assert.notEqual(pinDrawing(),pinBefore,'啟動曲柄必須讓圓銷沿圓周移動');
assert.deepEqual(canvasDrawing(),fixedBefore,'曲柄運轉不能令綠色件移位或縮放');
drive.events.get('click')();const pausedPin=pinDrawing();assert.equal(frames.size,0);
assert.equal(drive.textContent,'繼續');assert.equal(savedState.modelContent.version,13);
const actualPose=movingDrawing().map(m=>m[0]),analysisPin=pinDrawing(),analysisFixed=canvasDrawing();
elements.get('rubber-sim-analyze').events.get('click')();
assert.equal(frames.size,0);assert.deepEqual(movingDrawing().map(m=>m[0]),actualPose);
assert.equal(pinDrawing(),analysisPin);assert.deepEqual(canvasDrawing(),analysisFixed);
assert.ok(elements.get('rubber-sim-cycle-segments').content.length>0,'分析按鈕必須更新角度分段');
near(Number.parseFloat(elements.get('rubber-sim-cycle-marker').style.left),Number(elements.get('rubber-sim-angle').value)+50,1e-9,'分析白線須顯示白棍角度，不能顯示曲柄進度');
const analysisPanel=elements.get('rubber-sim-cycle-panel'),analysisButton=elements.get('rubber-sim-analyze');
assert.equal(analysisPanel.open,true,'分析按鈕須即時展開結果');
assert.equal(analysisButton['aria-expanded'],'true');
assert.equal(elements.get('rubber-sim-cycle-heading').focused,true,'鍵盤焦點須移到分析結果');
assert.equal(analysisPanel.scrollOptions.block,'nearest','結果須帶入目前視野');
assert.ok(elements.get('rubber-sim-status').textContent.startsWith('分析完成'));
analysisPanel.open=false;analysisPanel.events.get('toggle')();assert.equal(analysisButton['aria-expanded'],'false');

assert.ok(savedState.modelContent.crankAngle<90,'自動曲柄必須順時針轉動');
assert.notEqual(savedState.modelContent.crankAngle,s.designDefaults.crankAngle);
storageEvent({modelContent:{...savedState.modelContent,crankAngle:-20},privateContent:{control:'crankAngle'}});
near(Number(elements.get('rubber-sim-crankAngle').value),110,1e-9);
assert.notEqual(pinDrawing(),pausedPin,'保存的曲柄角度須可還原');
elements.get('rubber-sim-home').events.get('click')();
near(Number(elements.get('rubber-sim-crankAngle').value),0,1e-9);
assert.ok(!elements.has('rubber-sim-pinRadius')&&!elements.has('rubber-sim-crankRPM'),'曲柄須移除多餘的大小及轉速 slider');
assert.ok(!elements.has('rubber-sim-crankLength'),'O–Q 固定 21mm，不能再改長度');
assert.equal(visibleFields.filter(([key])=>key.startsWith('crank')).length,2,'曲柄只保留 O 圓心 X、Y 兩條 slider');
const rangeMin=elements.get('rubber-sim-rotationMin'),rangeMax=elements.get('rubber-sim-rotationMax');
const beforeLimitPreview=canvasDrawing(),pinBeforePreview=pinDrawing(),rodsBeforePreview=movingDrawing().map(m=>m[0]);
drive.events.get('click')();rangeMin.value='-30';rangeMin.events.get('input')();
assert.equal(frames.size,0,'預覽轉角下限時暫停模擬');
assert.equal(elements.get('rubber-sim-angle-value').value,'-30.0°','拖動下限時白棍須立即轉到下限');
assert.ok(elements.get('rubber-sim-status').textContent.includes('下限預覽'));
const lowerRods=movingDrawing().map(m=>m[0]);assert.notDeepEqual(lowerRods,rodsBeforePreview,'下限預覽必須真的改變 SVG 白棍位置');
rangeMax.value='20';rangeMax.events.get('input')();rangeMax.events.get('change')();
assert.equal(elements.get('rubber-sim-angle-value').value,'20.0°','拖動上限時白棍須立即轉到上限');
assert.notDeepEqual(movingDrawing().map(m=>m[0]),lowerRods,'上限預覽必須真的改變 SVG 白棍位置');
assert.deepEqual(canvasDrawing(),beforeLimitPreview,'預覽兩個邊界不能移動或縮放綠色固定件');
assert.equal(pinDrawing(),pinBeforePreview,'預覽白棍邊界不能轉動曲柄');
near(savedState.modelContent.angle,20,1e-9,'放開滑塊後保留正在預覽的姿勢');
assert.equal(savedState.modelContent.design.rotationMin,-30);assert.equal(savedState.modelContent.design.rotationMax,20);
assert.equal(elements.get('rubber-sim-range-window').style.marginInlineStart,'20%');assert.equal(elements.get('rubber-sim-range-window').style.width,'50%');
assert.equal(elements.get('rubber-sim-angle').min,-30);assert.equal(elements.get('rubber-sim-angle').max,20);
const constrainedAngle=elements.get('rubber-sim-angle');constrainedAngle.value='90';constrainedAngle.events.get('input')();
assert.equal(elements.get('rubber-sim-angle-value').value,'20.0°');
rangeMin.value='40';rangeMin.events.get('input')();assert.equal(Number(rangeMin.value),20,'左滑塊不能越過右滑塊');
elements.get('rubber-sim-home').events.get('click')();
rangeMax.value='90';rangeMax.events.get('input')();
assert.equal(elements.get('rubber-sim-angle-value').value,'50.0°','超出全範圍上限的輸入必須限制在 50°');
rangeMin.value='-90';rangeMin.events.get('input')();
assert.equal(elements.get('rubber-sim-angle-value').value,'-50.0°','超出全範圍下限的輸入必須限制在 −50°');
elements.get('rubber-sim-home').events.get('click')();
// 連續旋轉手勢用不接觸白棍的 O 配置；O 平移推棍另有實際拖動檢查。
storageEvent({modelContent:{version:13,angle:0,crankAngle:90,design:{...s.designDefaults,crankX:190,crankY:170}}});
const dragSvg=elements.get('.mechanism'),dragFixed=canvasDrawing();
let center=dragSvg.content.match(/<circle data-crank-center cx="([^"]+)" cy="([^"]+)"/).slice(1).map(Number);
const gesture=(type,bearing,id=1)=>dragSvg.events.get(type)({button:0,pointerId:id,clientX:center[0]+30*Math.cos(bearing),clientY:center[1]-30*Math.sin(bearing),target:{closest:selector=>selector==='[data-crank-handle]'?{}:null},preventDefault(){}});
let dragTimestamp=0;
function runDragFrame(){const [id,callback]=frames.entries().next().value;frames.delete(id);callback(dragTimestamp);dragTimestamp+=16;}
gesture('pointerdown',Math.PI/2);assert.equal(dragSvg.capturedPointer,1);runDragFrame();
gesture('pointerdown',0,2);gesture('pointermove',0,2);runDragFrame();near(Number(elements.get('rubber-sim-crankAngle').value),0,1e-6,'第二隻手指不能改變拖動');
for(let i=1;i<=72;i++){gesture('pointermove',Math.PI/2-i*Math.PI/12);runDragFrame();}
near(Number(elements.get('rubber-sim-crankAngle').value),1080,0.01,'順時針拖動三圈應累積角度，不能卡在 ±180°');
gesture('pointermove',Math.PI/2-Math.PI*6+Math.PI/12);runDragFrame();
near(Number(elements.get('rubber-sim-crankAngle').value),1080,0.01,'逆時針手勢不能令曲柄反轉');
assert.deepEqual(canvasDrawing(),dragFixed);assert.ok(!dragSvg.content.includes('data-control='),'手動拖動不覆蓋藍色提示');
gesture('pointerup',Math.PI/2+Math.PI*6);assert.equal(dragSvg.capturedPointer,undefined);
near(savedState.modelContent.crankAngle,-990,0.01,'須保存順時針連續轉動的完整角度');
const numberAngle=elements.get('rubber-sim-crankAngle');numberAngle.value='1170';numberAngle.events.get('change')();runDragFrame();
near(Number(numberAngle.value),1170,0.01,'可輸入累計多圈角度');
numberAngle.events.get('keydown')({key:'ArrowUp',preventDefault(){}});runDragFrame();near(Number(numberAngle.value),1171,0.01);
numberAngle.events.get('keydown')({key:'ArrowDown',preventDefault(){}});runDragFrame();near(Number(numberAngle.value),1171,0.01,'鍵盤不能逆轉曲柄');
numberAngle.value='1';numberAngle.events.get('change')();runDragFrame();near(Number(numberAngle.value),1171,0.01,'輸入較小累計角度亦不能逆轉');
elements.get('rubber-sim-home').events.get('click')();
rangeMin.value='-10';rangeMin.events.get('input')();rangeMax.value='10';rangeMax.events.get('input')();
center=dragSvg.content.match(/<circle data-crank-center cx="([^"]+)" cy="([^"]+)"/).slice(1).map(Number);
let bearing=Math.PI/2;gesture('pointerdown',bearing);runDragFrame();
for(let i=0;i<100&&!elements.get('rubber-sim-status').textContent.includes('P 已到轉角限制');i++){
  bearing-=Math.PI/36;gesture('pointermove',bearing);runDragFrame();
}
assert.ok(elements.get('rubber-sim-status').textContent.includes('P 已到轉角限制'),'手動推到限制時亦須停下');
assert.equal(dragSvg.capturedPointer,1,'到達擋位不能中斷手動拖動');
const stoppedPhase=Number(numberAngle.value);bearing+=Math.PI/18;gesture('pointermove',bearing);runDragFrame();
near(Number(numberAngle.value),stoppedPhase,0.01,'在擋位亦不能逆轉曲柄');
gesture('pointerup',bearing);elements.get('rubber-sim-home').events.get('click')();
storageEvent({modelContent:{version:13,angle:0,crankAngle:90,design:{...s.designDefaults,crankX:190,crankY:170}}});
const beforeMoveO=canvasDrawing();
const originalO=dragSvg.content.match(/<circle data-crank-center cx="([^"]+)" cy="([^"]+)"/).slice(1).map(Number);
const pixelsPerMm=Number(dragSvg.content.match(/<circle data-crank-pin[^>]* r="([^"]+)"/)[1])/3;
const moveO=(type,x,y,id=3)=>dragSvg.events.get(type)({button:0,pointerId:id,clientX:originalO[0]+(x-190)*pixelsPerMm+4,clientY:originalO[1]-(y-170)*pixelsPerMm+2,target:{closest:selector=>selector==='[data-crank-center-handle]'?{}:null},preventDefault(){}});
drive.events.get('click')();assert.ok(frames.size>0);
moveO('pointerdown',190,170);assert.equal(dragSvg.capturedPointer,3);assert.equal(frames.size,0,'拖動 O 時先暫停曲柄');
near(Number(elements.get('rubber-sim-crankX').value),190,1e-9,'在 O 旁邊按下不能令圓心跳位');
moveO('pointermove',0,24,4);near(Number(elements.get('rubber-sim-crankX').value),190,1e-9,'拖 O 亦須忽略第二隻手指');
moveO('pointermove',200,160);
near(Number(elements.get('rubber-sim-crankX').value),200,1e-6);near(Number(elements.get('rubber-sim-crankY').value),160,1e-6);
const movedO=dragSvg.content.match(/<circle data-crank-center cx="([^"]+)" cy="([^"]+)"/).slice(1).map(Number);
const movedQ=dragSvg.content.match(/<circle data-crank-pin cx="([^"]+)" cy="([^"]+)"/).slice(1).map(Number);
near(distance(movedO,movedQ),21*pixelsPerMm,1e-6,'拖 O 時 Q 必須跟隨且保持 21mm');
assert.deepEqual(canvasDrawing(),beforeMoveO,'拖 O 不能移動或縮放綠色固定件');
moveO('pointermove',200,-50);near(Number(elements.get('rubber-sim-crankY').value),24,1e-6,'拖 O 低過底板上方 21mm 時必須停在邊界');
moveO('pointerup',200,-50);assert.equal(dragSvg.capturedPointer,undefined);
near(savedState.modelContent.design.crankX,200,1e-6);near(savedState.modelContent.design.crankY,24,1e-6);near(savedState.modelContent.design.crankLength,21,1e-12);

const pointerO=(type,point)=>{
  const origin=canvasDrawing().green[0].match(/d="M([^ ]+)/)[1].split(',').map(Number);
  const scale=Number(dragSvg.content.match(/<circle data-crank-pin[^>]* r="([^"]+)"/)[1])/3;
  dragSvg.events.get(type)({button:0,pointerId:9,clientX:viewportLeft+origin[0]+point[0]*scale+4,clientY:viewportTop+origin[1]-point[1]*scale+2,target:{closest:selector=>selector==='[data-crank-center-handle]'?{}:null},preventDefault(){}});
};
storageEvent({modelContent:{version:13,angle:0,crankAngle:90,design:slideGeometry.design}});
const pushFixed=canvasDrawing(),pushRods=movingDrawing().map(m=>m[0]);
pointerO('pointerdown',slideStart);pointerO('pointermove',slideTarget);
assert.ok(Math.abs(Number(elements.get('rubber-sim-angle').value))>1,'拖 O 令 Q 碰到白棍時須立即轉動白棍');
assert.notDeepEqual(movingDrawing().map(m=>m[0]),pushRods,'拖 O 必須真的改變 SVG 的白棍位置');
assert.deepEqual(canvasDrawing(),pushFixed);assert.equal(Number(elements.get('rubber-sim-crankAngle').value),0,'拖 O 不會改曲柄轉角');
assert.ok(elements.get('rubber-sim-status').textContent.includes('Q 推動白棍'));
pointerO('pointerup',slideTarget);
near(savedState.modelContent.crankAngle,90,1e-12);
for(const c of s.pinContacts(savedState.modelContent.angle*Math.PI/180,Math.PI/2,s.buildGeometry(savedState.modelContent.design)))assert.ok(c.gap>=-.001);

storageEvent({modelContent:{version:13,angle:0,crankAngle:90,design:slideGeometry.design}});
const xPush=elements.get('rubber-sim-crankX');xPush.value=String(slideTarget[0]);xPush.events.get('input')();xPush.events.get('change')();
assert.ok(Math.abs(savedState.modelContent.angle)>1,'O 左右 slider 亦須推動白棍');
assert.deepEqual(canvasDrawing(),pushFixed);
for(const c of s.pinContacts(savedState.modelContent.angle*Math.PI/180,Math.PI/2,s.buildGeometry(savedState.modelContent.design)))assert.ok(c.gap>=-.001);

const arm2Probe=s.along(g0.arms[1],.8),u2=g0.arms[1][1].map((v,i)=>(v-g0.arms[1][0][i])/114),n2Probe=[-u2[1],u2[0]];
const yStart=arm2Probe.map((v,i)=>v-n2Probe[i]*12-(i?21:0));
storageEvent({modelContent:{version:13,angle:0,crankAngle:90,design:{...s.designDefaults,crankX:yStart[0],crankY:yStart[1]}}});
const yPush=elements.get('rubber-sim-crankY');yPush.value=String(yStart[1]+16);yPush.events.get('input')();yPush.events.get('change')();
assert.ok(Math.abs(savedState.modelContent.angle)>1,'O 上下 slider 亦須推動白棍');
assert.deepEqual(canvasDrawing(),pushFixed);
for(const c of s.pinContacts(savedState.modelContent.angle*Math.PI/180,Math.PI/2,s.buildGeometry(savedState.modelContent.design)))assert.ok(c.gap>=-.001);

storageEvent({modelContent:{version:13,angle:0,crankAngle:90,design:{...slideGeometry.design,rotationMin:-2,rotationMax:2}}});
pointerO('pointerdown',slideStart);pointerO('pointermove',slideTarget);
assert.ok(elements.get('rubber-sim-status').textContent.includes('P 已到轉角限制'));
assert.equal(dragSvg.capturedPointer,9,'推到 P 擋位後仍保留 O 拖動');
pointerO('pointerup',slideTarget);
assert.ok(savedState.modelContent.angle>=-2&&savedState.modelContent.angle<=2);
assert.ok(distance([savedState.modelContent.design.crankX,savedState.modelContent.design.crankY],slideTarget)>1);
for(const c of s.pinContacts(savedState.modelContent.angle*Math.PI/180,Math.PI/2,s.buildGeometry(savedState.modelContent.design)))assert.ok(c.gap>=-.001);

elements.get('rubber-sim-home').events.get('click')();
const pivotSlider=elements.get('rubber-sim-pivotT'),beamPivotSlider=elements.get('rubber-sim-beamT'),wholeSlider=elements.get('rubber-sim-angle');
pivotSlider.value='5';pivotSlider.events.get('input')();
assert.deepEqual(canvasDrawing(),fixedBefore,'調 P 在活動棍的位置不能改變綠色件的畫面座標、大小或畫布高度');
assert.notDeepEqual(movingDrawing().map(match=>match[0]),movingBefore,'活動棍仍須正常移位');
let canvasCases=0;
for(const width of [320,736]){
  elements.get('rubber-sim-home').events.get('click')();notifyResize(width,0);
  for(const [id,draw] of [...frames]){frames.delete(id);draw();}
  const fixed=canvasDrawing();assert.equal(fixed.green.length,4);
  for(const pivotT of [5,50,95])for(const beamT of [5,50,95])for(const angle of [-50,-25,0,25,50]){
    for(const [slider,value] of [[pivotSlider,pivotT],[beamPivotSlider,beamT],[wholeSlider,angle]]){slider.value=String(value);slider.events.get('input')();}
    assert.deepEqual(canvasDrawing(),fixed,'P 移位及整組轉動不能自動縮放或置中綠色件');
    const [,,svgWidth,svgHeight]=fixed.viewBox.split(' ').map(Number),paths=movingDrawing();assert.equal(paths.length,2);
    for(const match of paths){
      const ends=match[1].match(/^M(.+)L(.+)$/),r=Number(match[2])/2;assert.ok(ends);
      for(const end of ends.slice(1).map(value=>value.split(',').map(Number))){
        assert.ok(end[0]-r>=-1e-9&&end[0]+r<=svgWidth+1e-9&&end[1]-r>=-1e-9&&end[1]+r<=svgHeight+1e-9,'P 的全程移位及轉動須留在固定視野內');
      }
    }
    canvasCases++;
  }
}
elements.get('rubber-sim-home').events.get('click')();
const fixedGreen=canvasDrawing();
for(const key of ['join1','join2','angle2']){
  const slider=elements.get('rubber-sim-'+key);slider.value=String(slider.min);slider.events.get('input')();
  assert.deepEqual(canvasDrawing(),fixedGreen,'改兩棍接合位置或夾角不能令固定件縮放');
}
elements.get('rubber-sim-home').events.get('click')();
const greenSlider=elements.get('rubber-sim-beamX');greenSlider.value=String(s.designDefaults.beamX+10);greenSlider.events.get('input')();
const movedGreen=canvasDrawing();assert.notDeepEqual(movedGreen.green,fixedGreen.green,'直接調整 G 時綠棍仍可移動');
assert.equal(movedGreen.viewBox,fixedGreen.viewBox);assert.equal(movedGreen.height,fixedGreen.height);
assert.equal(movedGreen.green[0],fixedGreen.green[0],'移動 G 不能令固定底板在畫面上移位或縮放');
storageEvent({modelContent:{version:10,angle:16.3,design:{...s.designDefaults,beamX:450}},privateContent:{control:'beamX'}});
assert.equal(elements.get('rubber-sim-angle-value').value,'16.3°','新版本保存的調整仍須正常還原');
const xSlider=elements.get('rubber-sim-beamX');
assert.ok(Number(xSlider.value)<210&&Number(xSlider.max)<210,'舊設定及 slider 範圍必須限制在 210 mm 底板內');
xSlider.events.get('change')();checkAttachments(s.buildGeometry(savedState.modelContent.design));
checkRubberSvg();
elements.get('rubber-sim-home').events.get('click')();notifyResize(1200,0);
for(const [id,draw] of [...frames]){frames.delete(id);draw();}
const desktopCanvas=canvasDrawing();
assert.ok(Number.parseFloat(desktopCanvas.height)>400,'大視窗的機構圖不再受 400px 高度限制');
const desktopRod=movingDrawing()[0],rodEnds=desktopRod[1].match(/^M(.+)L(.+)$/).slice(1).map(p=>p.split(',').map(Number));
const desktopStickPixels=distance(...rodEnds)+Number(desktopRod[2]);
assert.ok(desktopStickPixels>230,'大視窗的 114mm 白棍必須實際放大');
runtime.window.innerHeight=1200;globals.get('resize')();
for(const [id,draw] of [...frames]){frames.delete(id);draw();}
assert.ok(Number.parseFloat(canvasDrawing().height)>Number.parseFloat(desktopCanvas.height),'視窗加高亦須放大機構圖');
const resizedCanvas=canvasDrawing();pivotSlider.value='10';pivotSlider.events.get('input')();
assert.deepEqual(canvasDrawing(),resizedCanvas,'放大後調 P 亦不能縮放固定件');
// 注入四個平板畫布格尺寸，檢查實際繪圖及事件；這不是 Safari／CSS 實機驗收。
const tabletCases=[];
for(const [screenWidth,screenHeight,canvasWidth,slotHeight] of [[1180,820,804,560],[1194,834,818,574],[820,1180,780,397],[834,1194,794,404]]){
  runtime.window.innerWidth=screenWidth;runtime.window.innerHeight=screenHeight;
  viewportLeft=20;viewportTop=160;
  notifyResize(canvasWidth,slotHeight);globals.get('resize')();
  for(const [id,draw] of [...frames]){frames.delete(id);draw();}
  storageEvent({modelContent:{version:13,angle:0,crankAngle:90,design:s.designDefaults}});
  const fixed=canvasDrawing();
  near(Number.parseFloat(fixed.height),slotHeight,1e-9,'平板 SVG 要填滿分配的畫布格');
  assert.equal(Number(fixed.viewBox.split(' ')[3]),slotHeight);
  assert.ok(/data-crank-center-handle>[\s\S]*?r="22" fill="transparent"/.test(dragSvg.content),'O 的觸控直徑至少 44px');
  for(const angle of [-50,0,50]){
    wholeSlider.value=String(angle);wholeSlider.events.get('input')();
    assert.deepEqual(canvasDrawing(),fixed,'平板調白棍角度不能令固定件縮放或移位');
    for(const match of movingDrawing()){
      const radius=Number(match[2])/2;
      for(const [x,y] of match[1].match(/^M(.+)L(.+)$/).slice(1).map(p=>p.split(',').map(Number))){
        assert.ok(x-radius>=0&&x+radius<=canvasWidth&&y-radius>=0&&y+radius<=slotHeight,'平板白棍須完整留在畫布格內');
      }
    }
  }
  storageEvent({modelContent:{version:13,angle:0,crankAngle:90,design:slideGeometry.design}});
  const fixedBeforeDrag=canvasDrawing();
  pointerO('pointerdown',slideStart);pointerO('pointermove',slideTarget);pointerO('pointerup',slideTarget);
  assert.ok(Math.abs(Number(elements.get('rubber-sim-angle').value))>1,'平板畫布有邊距及垂直置中時，拖 O 仍須推動白棍');
  assert.deepEqual(canvasDrawing(),fixedBeforeDrag);
  assert.equal(frames.size,0);
  notifyResize(canvasWidth,slotHeight+40);assert.equal(frames.size,1,'平板畫布格改高須安排重繪');
  for(const [id,draw] of [...frames]){frames.delete(id);draw();}
  near(Number.parseFloat(canvasDrawing().height),slotHeight+40,1e-9);
  notifyResize(canvasWidth,slotHeight+40);assert.equal(frames.size,0,'同一尺寸通知不能循環重繪');
  const poseBeforeAnalysis=JSON.stringify(savedState.modelContent),canvasBeforeAnalysis=canvasDrawing();
  analysisButton.events.get('click')();
  assert.equal(JSON.stringify(savedState.modelContent),poseBeforeAnalysis,'平板展開分析亦不能改動機構姿勢');
  assert.deepEqual(canvasDrawing(),canvasBeforeAnalysis);
  tabletCases.push({viewport:[screenWidth,screenHeight],canvas:[canvasWidth,slotHeight]});
}
runtime.window.innerWidth=1600;runtime.window.innerHeight=1200;viewportLeft=viewportTop=0;
notifyResize(1200,0);globals.get('resize')();
for(const [id,draw] of [...frames]){frames.delete(id);draw();}
elements.get('rubber-sim-home').events.get('click')();
// 初始配置獨立保存；一般試調、還原及重新載入都不能改寫它。
const customDesign={...s.designDefaults,beamX:145,beamY:118,hookAngle:75,pivotT:58,beamT:40,join1:32,join2:61,angle2:-42,crankX:190,crankY:170,rotationMin:-35,rotationMax:45};
storageEvent({modelContent:{version:13,angle:23.4,crankAngle:-20,design:customDesign}});
elements.get('rubber-sim-drive').events.get('click')();assert.ok(frames.size>0);
elements.get('rubber-sim-store-initial').events.get('click')();assert.equal(frames.size,0,'保存初始配置時須暫停運動');
assert.equal(elements.get('rubber-sim-status').textContent,'已保存為初始配置');
const initialRaw=storageValues.get(initialStorageKey),customInitial=JSON.parse(initialRaw).modelContent;
for(const [key,value] of Object.entries(customDesign))near(customInitial.design[key],value,1e-12);
near(customInitial.angle,23.4,1e-12);near(customInitial.crankAngle,-20,1e-12);
const editValue=(key,value)=>{const slider=elements.get(`rubber-sim-${key}`);slider.value=String(value);slider.events.get('input')();slider.events.get('change')();};
editValue('beamX',155);editValue('angle',-12);
assert.equal(storageValues.get(initialStorageKey),initialRaw,'一般自動保存不能覆蓋初始配置');
elements.get('rubber-sim-home').events.get('click')();
near(Number(elements.get('rubber-sim-beamX').value),145,1e-12);
assert.equal(elements.get('rubber-sim-angle-value').value,'23.4°');near(Number(elements.get('rubber-sim-crankAngle').value),110,1e-12);
editValue('beamX',150);editValue('angle',5);
const reload=()=>{frames.clear();guideTimers.clear();vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1],runtime);};
reload();
for(const [key,value] of Object.entries(customDesign)){const slider=elements.get(`rubber-sim-${key}`);if(slider&&key!=='crankAngle')near(Number(slider.value),value,1e-12);}
assert.equal(elements.get('rubber-sim-angle-value').value,'23.4°');near(Number(elements.get('rubber-sim-crankAngle').value),110,1e-12);
elements.get('rubber-sim-beamX').events.get('change')();
for(const key of ['angle','crankAngle'])near(savedState.modelContent[key],customInitial[key],1e-12,'重新載入須恢復保存的白棍及曲柄角度');
editValue('hookAngle',100);editValue('angle',30);elements.get('rubber-sim-store-initial').events.get('click')();
const replacedInitial=storageValues.get(initialStorageKey);assert.notEqual(replacedInitial,initialRaw,'再次按保存須更新初始配置');
editValue('hookAngle',110);editValue('angle',40);
const writeStorage=localStorage.setItem;
localStorage.setItem=(key,value)=>{if(key===initialStorageKey)throw new Error('儲存被拒絕');writeStorage(key,value);};
elements.get('rubber-sim-store-initial').events.get('click')();
assert.ok(elements.get('rubber-sim-status').textContent.includes('無法保存初始配置'));
assert.equal(storageValues.get(initialStorageKey),replacedInitial,'保存失敗不能覆蓋原有初始配置');
localStorage.setItem=writeStorage;elements.get('rubber-sim-home').events.get('click')();
near(Number(elements.get('rubber-sim-hookAngle').value),100,1e-12);assert.equal(elements.get('rubber-sim-angle-value').value,'30.0°');
editValue('hookAngle',120);editValue('angle',45);storageValues.set(initialStorageKey,'錯誤 JSON');reload();
near(Number(elements.get('rubber-sim-hookAngle').value),120,1e-12);assert.equal(elements.get('rubber-sim-angle-value').value,'45.0°');
elements.get('rubber-sim-home').events.get('click')();
for(const [key,value] of Object.entries(screenshotDefaults))near(Number(elements.get(`rubber-sim-${key}`).value),value,1e-12);
assert.equal(elements.get('rubber-sim-angle-value').value,'0.0°','沒有有效初始配置時仍可還原原始預設');
// 更新前的初始配置要保留部件尺寸及位置，只收窄越界角度。
const legacyInitial=JSON.stringify({modelContent:{version:13,angle:90,crankAngle:-20,design:{...customDesign,rotationMin:-180,rotationMax:180}},privateContent:{control:'join1'}});
storageValues.set(initialStorageKey,legacyInitial);reload();
for(const [key,value] of Object.entries(customDesign)){
  const slider=elements.get(`rubber-sim-${key}`);if(slider&&!['rotationMin','rotationMax','crankAngle'].includes(key))near(Number(slider.value),value,1e-12);
}
assert.equal(elements.get('rubber-sim-angle-value').value,'50.0°');
assert.equal(Number(elements.get('rubber-sim-rotationMin').value),-50);assert.equal(Number(elements.get('rubber-sim-rotationMax').value),50);
assert.equal(elements.get('rubber-sim-select-pivot')['aria-pressed'],'true');
assert.equal(storageValues.get(initialStorageKey),legacyInitial,'收窄角度不能覆蓋原有保存資料');
elements.get('rubber-sim-home').events.get('click')();assert.equal(elements.get('rubber-sim-angle-value').value,'50.0°');
storageValues.delete(initialStorageKey);elements.get('rubber-sim-home').events.get('click')();
// 用真正的 Q 指標事件比較停住／轉動，排除白棍只因橡筋自行轉動而造成的假通過。
function manualContactTurn(degreesPerStep,turn){
  elements.get('rubber-sim-home').events.get('click')();
  const fixed=canvasDrawing();center=dragSvg.content.match(/<circle data-crank-center cx="([^"]+)" cy="([^"]+)"/).slice(1).map(Number);
  const radius=Number(dragSvg.content.match(/<circle data-crank-pin[^>]* r="([^"]+)"/)[1])/3*21;
  const pointer=(type,bearing)=>dragSvg.events.get(type)({button:0,pointerId:7,clientX:center[0]+radius*Math.cos(bearing),clientY:center[1]-radius*Math.sin(bearing),target:{closest:selector=>selector==='[data-crank-handle]'?{}:null},preventDefault(){}});
  pointer('pointerdown',Math.PI/2);runDragFrame();let contacts=0;
  for(let i=1;i<=90/degreesPerStep;i++){
    pointer('pointermove',Math.PI/2-(turn?i*degreesPerStep:0)*Math.PI/180);runDragFrame();
    if(elements.get('rubber-sim-values').textContent.includes('接觸棍'))contacts++;
  }
  pointer('pointerup',Math.PI/2-(turn?90:0)*Math.PI/180);
  assert.deepEqual(canvasDrawing(),fixed,'手動推棍不能移動或縮放固定件');
  near(Number(elements.get('rubber-sim-crankAngle').value),turn?90:0,.01);
  for(const contact of s.pinContacts(savedState.modelContent.angle*Math.PI/180,savedState.modelContent.crankAngle*Math.PI/180))assert.ok(contact.gap>=-.001,'手動轉動後 Q 不得穿透白棍');
  return {angle:savedState.modelContent.angle,contacts};
}
for(const stepDegrees of [3,15]){
  const held=manualContactTurn(stepDegrees,false),turned=manualContactTurn(stepDegrees,true);
  assert.ok(turned.contacts>0,'手動曲柄必須碰到白棍');
  assert.ok(Math.abs(turned.angle-held.angle)>1,'手動轉 Q 必須改變白棍角度，不能只有橡筋自行轉動');
}
// 已到 P 擋位的配置亦須有可見結果及原因，不能讓按鈕看似沒有反應。
storageEvent({modelContent:{version:13,angle:0,crankAngle:90,design:limitedGeometry.design}});
const blockedPose=movingDrawing().map(m=>m[0]);analysisPanel.open=false;
analysisButton.events.get('click')();
assert.equal(analysisPanel.open,true);assert.ok(elements.get('rubber-sim-status').textContent.includes('P 已到轉角限制'));
assert.ok(elements.get('rubber-sim-cycle-ranges').content.includes('未到達的角度'));
assert.deepEqual(movingDrawing().map(m=>m[0]),blockedPose);
console.log(JSON.stringify({result:'通過',tabletCases,initialConfiguration:screenshotDefaults,whiteRodRangeDegrees:[-50,50],whiteSwingDegrees:cycleReport.segments.map(s=>({phase:s.kind,from:s.fromAngle,to:s.toAngle,min:s.minAngle,max:s.maxAngle})),desktopStickPixels,crank:{contactSteps,clearSteps,strokeDegrees:(maxAngle-minAngle)*180/Math.PI,cycleSegments:cycleReport.segments},stickSizeMm:[114,10],baseLengthMm:210,sliders:visibleFields.length+1,geometryVariants:variants.length,attachmentSweep:292,notchSweep,canvasCases,energyDrift,checks:['平板橫直四尺寸／畫布格高度及置中／帶邊距拖 O 推棍／分析不縮放／尺寸通知不循環','原生獨立 HTML／沒有 iframe 或 ChatGPT 依賴／本機配置相容','大視窗圖像放大／高度調整／調 P 時固定件不縮放','曲柄只順時針／連續三圈／逆向手勢及輸入不反轉／拖 O 與高度限制','拖 O 及 X/Y slider 推白棍／大幅移動不穿透／P 擋位停止 O','白棍集中 ±50°／P 面板預設／舊保存角度收窄／雙滑塊預覽／碰擋位時停止','分析按鈕展開結果／焦點與捲動／完成提示／無凹位或擋位亦顯示原因',
'白棍起止及極值角度／繞 P 掃過實際棍身範圍／回拉與推動分色／分析不改姿勢','曲柄圓周運動／推棍接觸／離開後不拉棍／阻擋 P 時停下','圓銷不穿棍／啟動與暫停／曲柄角度保存','截圖九值初始／還原／新設定保存','舊快照不覆蓋新初始配置','固定視野／P 移位不改變綠色件座標及大小','P 全程移位及旋轉留在畫布內／G 可直接調整','A–B 直線連接／拉力按直線長度','SVG 只含 A、B 兩端／無繞棍路徑','三支長棍同尺寸／底板 210 mm','支架及 slider 設定限制在底板內','上方外沿交點 B／與 J 分開／轉動後保留同一交點','外沿接觸／沒有凹位時停止放手','底板鎖定','slider 與圖形標記同步','部件切換／移位箭嘴／轉角弧線','尺寸通知不循環重繪','本機儲存失敗／損壞資料處理','保存新初始配置／還原及重新載入／試調不覆蓋／失敗保留原值','A 棍與底板上邊凹角／改棍角度重算交界／力矩與能量同步','支架貼住圓頭棍／連接底板','雙棍接合與慣量','力矩／能量／阻尼停定'],settledDegrees:q[0]*180/Math.PI}));
