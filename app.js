"use strict";
/* Moteur physique 2D maison : corps rigides (boîtes et cercles), collisions SAT,
   frottements, rebonds, liaisons (points fixes, cordes, ressorts, soudures).
   Il expose l'objet global `Matter` avec l'API utilisée par app.js, sans aucune dépendance externe. */
(function(){
const K=277.78;            // conversion force -> accélération par pas de 1/60 s
let ID=1,GRP=0;
const Z={x:0,y:0};
const V={add:(a,b)=>({x:a.x+b.x,y:a.y+b.y}),sub:(a,b)=>({x:a.x-b.x,y:a.y-b.y}),
  rotate:(v,a)=>{const c=Math.cos(a),s=Math.sin(a);return{x:v.x*c-v.y*s,y:v.x*s+v.y*c}},magnitude:v=>Math.hypot(v.x,v.y)};
const cl=(v,a,b)=>v<a?a:v>b?b:v;

/* ---- Composite : conteneur de corps, de liaisons et de sous-conteneurs ---- */
const Composite={
  create:()=>({isComposite:1,bodies:[],constraints:[],composites:[]}),
  add(c,x){if(Array.isArray(x))x.forEach(i=>Composite.add(c,i));else if(x.isComposite)c.composites.push(x);
    else if(x.type==='constraint')c.constraints.push(x);else c.bodies.push(x);return c},
  remove(c,x){if(Array.isArray(x))return x.forEach(i=>Composite.remove(c,i));
    for(const k of['bodies','constraints','composites']){const i=c[k].indexOf(x);if(i>=0)c[k].splice(i,1)}
    c.composites.forEach(s=>Composite.remove(s,x))},
  clear(c){c.bodies=[];c.constraints=[];c.composites=[]},
  allBodies:c=>c.bodies.concat(...c.composites.map(Composite.allBodies)),
  allConstraints:c=>c.constraints.concat(...c.composites.map(Composite.allConstraints))};

/* ---- Corps ---- */
function mk(x,y,o,r,w,h){
  const m=(r?Math.PI*r*r:w*h)*(o.density||.001),I=r?m*r*r/2:m*(w*w+h*h)/12;
  return{type:'body',id:ID++,position:{x,y},angle:o.angle||0,velocity:{x:0,y:0},angularVelocity:0,force:{x:0,y:0},torque:0,
    mass:m,inertia:I,invMass:o.isStatic?0:1/m,invInertia:o.isStatic?0:1/I,isStatic:!!o.isStatic,
    friction:o.friction??.1,restitution:o.restitution??0,frictionAir:o.frictionAir??.01,
    collisionFilter:Object.assign({group:0},o.collisionFilter),plugin:{},r:r||0,hw:(w||0)/2,hh:(h||0)/2}}
const Bodies={rectangle:(x,y,w,h,o={})=>mk(x,y,o,0,w,h),circle:(x,y,r,o={})=>mk(x,y,o,r)};
const Body={
  nextGroup:()=>--GRP,
  setStatic(b,s){b.isStatic=s;b.invMass=s?0:1/b.mass;b.invInertia=s?0:1/b.inertia;if(s){b.velocity.x=b.velocity.y=b.angularVelocity=0}},
  setMass(b,m){b.inertia*=m/b.mass;b.mass=m;if(!b.isStatic){b.invMass=1/m;b.invInertia=1/b.inertia}},
  setAngle:(b,a)=>{b.angle=a},
  setVelocity:(b,v)=>{b.velocity.x=v.x;b.velocity.y=v.y},
  setAngularVelocity:(b,w)=>{b.angularVelocity=w},
  applyForce(b,p,f){b.force.x+=f.x;b.force.y+=f.y;b.torque+=(p.x-b.position.x)*f.y-(p.y-b.position.y)*f.x}};

/* ---- Liaisons ---- */
function anch(c){const A=c.bodyA,B=c.bodyB,rA=A?V.rotate(c.pointA,A.angle):Z,rB=B?V.rotate(c.pointB,B.angle):Z;
  return[A?V.add(A.position,rA):c.pointA,B?V.add(B.position,rB):c.pointB,rA,rB]}
const Constraint={create(o){
  const c={type:'constraint',bodyA:o.bodyA||null,bodyB:o.bodyB||null,pointA:o.pointA||{x:0,y:0},pointB:o.pointB||{x:0,y:0},
    stiffness:o.stiffness??1,damping:o.damping||0,length:o.length};
  if(c.length===undefined){const[a,b]=anch(c);c.length=Math.hypot(b.x-a.x,b.y-a.y)}return c}};
const Query={point:(bs,p)=>bs.filter(b=>{
  if(b.r)return Math.hypot(p.x-b.position.x,p.y-b.position.y)<=b.r;
  const l=V.rotate(V.sub(p,b.position),-b.angle);return Math.abs(l.x)<=b.hw&&Math.abs(l.y)<=b.hh})};
const Events={on(e,n,f){(e._ev[n]=e._ev[n]||[]).push(f)}};

/* ---- Détection de collisions ---- */
const vs=b=>{const c=Math.cos(b.angle),s=Math.sin(b.angle),w=b.hw,h=b.hh;
  return[[-w,-h],[w,-h],[w,h],[-w,h]].map(([a,d])=>({x:b.position.x+a*c-d*s,y:b.position.y+a*s+d*c}))};
const inb=(p,b)=>{const l=V.rotate(V.sub(p,b.position),-b.angle);return Math.abs(l.x)<=b.hw+.6&&Math.abs(l.y)<=b.hh+.6};
function boxbox(a,b){
  const va=vs(a),vb=vs(b),ax=[[Math.cos(a.angle),Math.sin(a.angle)],[-Math.sin(a.angle),Math.cos(a.angle)],[Math.cos(b.angle),Math.sin(b.angle)],[-Math.sin(b.angle),Math.cos(b.angle)]];
  let best=1e9,n;
  for(const u of ax){let a0=1e9,a1=-1e9,b0=1e9,b1=-1e9;
    for(const p of va){const d=p.x*u[0]+p.y*u[1];if(d<a0)a0=d;if(d>a1)a1=d}
    for(const p of vb){const d=p.x*u[0]+p.y*u[1];if(d<b0)b0=d;if(d>b1)b1=d}
    const o=Math.min(a1,b1)-Math.max(a0,b0);if(o<=0)return null;if(o<best){best=o;n=u}}
  let nx=n[0],ny=n[1];if((b.position.x-a.position.x)*nx+(b.position.y-a.position.y)*ny<0){nx=-nx;ny=-ny}
  const pts=vb.filter(p=>inb(p,a)).concat(va.filter(p=>inb(p,b)));
  if(!pts.length){let m=1e9,q;for(const p of vb){const d=p.x*nx+p.y*ny;if(d<m){m=d;q=p}}pts.push(q)}
  return{n:{x:nx,y:ny},depth:best,pts}}
function circBox(c,x){   // normale orientée de la boîte vers le cercle
  const d=V.rotate(V.sub(c.position,x.position),-x.angle),cx=cl(d.x,-x.hw,x.hw),cy=cl(d.y,-x.hh,x.hh),dx=d.x-cx,dy=d.y-cy,q=dx*dx+dy*dy;
  if(q>c.r*c.r)return null;let nl,dep,pl;
  if(q>1e-9){const t=Math.sqrt(q);nl={x:dx/t,y:dy/t};dep=c.r-t;pl={x:cx,y:cy}}
  else{const px=x.hw-Math.abs(d.x),py=x.hh-Math.abs(d.y);
    if(px<py){const s=d.x<0?-1:1;nl={x:s,y:0};dep=px+c.r;pl={x:s*x.hw,y:d.y}}else{const s=d.y<0?-1:1;nl={x:0,y:s};dep=py+c.r;pl={x:d.x,y:s*x.hh}}}
  return{n:V.rotate(nl,x.angle),depth:dep,pts:[V.add(x.position,V.rotate(pl,x.angle))]}}
function collide(a,b){
  if(a.r&&b.r){const dx=b.position.x-a.position.x,dy=b.position.y-a.position.y,d=Math.hypot(dx,dy),s=a.r+b.r;if(d>=s)return null;
    const n=d>1e-9?{x:dx/d,y:dy/d}:{x:0,y:-1};return{n,depth:s-d,pts:[{x:a.position.x+n.x*a.r,y:a.position.y+n.y*a.r}]}}
  if(a.r){const k=circBox(a,b);if(!k)return null;k.n={x:-k.n.x,y:-k.n.y};return k}
  if(b.r)return circBox(b,a);
  return boxbox(a,b)}

/* ---- Résolution par impulsions ---- */
const D={position:Z,velocity:{x:0,y:0},angularVelocity:0,invMass:0,invInertia:0};
function imp(A,B,rA,rB,px,py){
  A.velocity.x-=px*A.invMass;A.velocity.y-=py*A.invMass;A.angularVelocity-=A.invInertia*(rA.x*py-rA.y*px);
  B.velocity.x+=px*B.invMass;B.velocity.y+=py*B.invMass;B.angularVelocity+=B.invInertia*(rB.x*py-rB.y*px)}
const rv=(a,b,p)=>[b.velocity.x-b.angularVelocity*p.rB.y-a.velocity.x+a.angularVelocity*p.rA.y,b.velocity.y+b.angularVelocity*p.rB.x-a.velocity.y-a.angularVelocity*p.rA.x];
function joint(c){
  const A=c.bodyA||D,B=c.bodyB||D;if(!A.invMass&&!B.invMass&&!A.invInertia&&!B.invInertia)return;
  const[pa,pb,rA,rB]=anch(c),ex=pb.x-pa.x,ey=pb.y-pa.y,d=Math.hypot(ex,ey);
  const soft=c.stiffness<.5,bi=soft?c.stiffness:Math.min(c.stiffness,.35),vw=soft?c.damping:1;
  const vx=B.velocity.x-B.angularVelocity*rB.y-A.velocity.x+A.angularVelocity*rA.y,vy=B.velocity.y+B.angularVelocity*rB.x-A.velocity.y-A.angularVelocity*rA.x;
  const mA=A.invMass,mB=B.invMass,iA=A.invInertia,iB=B.invInertia;
  if(c.length>.5&&d>1e-6){
    const nx=ex/d,ny=ey/d,cA=rA.x*ny-rA.y*nx,cB=rB.x*ny-rB.y*nx,k=mA+mB+iA*cA*cA+iB*cB*cB;if(!k)return;
    const l=-((vx*nx+vy*ny)*vw+bi*(d-c.length))/k;imp(A,B,rA,rB,nx*l,ny*l)}
  else{
    const k11=mA+mB+iA*rA.y*rA.y+iB*rB.y*rB.y,k22=mA+mB+iA*rA.x*rA.x+iB*rB.x*rB.x,k12=-iA*rA.x*rA.y-iB*rB.x*rB.y,det=k11*k22-k12*k12;
    if(Math.abs(det)<1e-12)return;const bx=-(vx*vw+bi*ex),by=-(vy*vw+bi*ey);
    imp(A,B,rA,rB,(k22*bx-k12*by)/det,(k11*by-k12*bx)/det)}}

/* ---- Pas de simulation ---- */
const Engine={
  create:()=>({world:Composite.create(),gravity:{x:0,y:1,scale:.001},timing:{timeScale:1},_ev:{},_p:new Set()}),
  update(e){
    const s=e.timing.timeScale,bs=Composite.allBodies(e.world),cs=Composite.allConstraints(e.world),g=e.gravity.y*e.gravity.scale*K*s;
    for(const b of bs){
      if(b.invMass){
        b.velocity.x+=b.force.x*b.invMass*K*s;b.velocity.y+=b.force.y*b.invMass*K*s+g;b.angularVelocity+=b.torque*b.invInertia*K*s;
        const f=1-b.frictionAir*s;b.velocity.x*=f;b.velocity.y*=f;b.angularVelocity*=f;
        const sp=Math.hypot(b.velocity.x,b.velocity.y);if(sp>25){b.velocity.x*=25/sp;b.velocity.y*=25/sp}
        b.angularVelocity=cl(b.angularVelocity,-.6,.6)}
      b.force.x=b.force.y=b.torque=0;
      const c=Math.abs(Math.cos(b.angle)),sn=Math.abs(Math.sin(b.angle)),ex=b.r||(c*b.hw+sn*b.hh),ey=b.r||(sn*b.hw+c*b.hh);
      b.bb=[b.position.x-ex,b.position.y-ey,b.position.x+ex,b.position.y+ey]}
    const cts=[],seen=new Set(),nw=[];
    for(let i=0;i<bs.length;i++){const a=bs[i];
      for(let j=i+1;j<bs.length;j++){const b=bs[j];
        if(!a.invMass&&!b.invMass)continue;
        const ga=a.collisionFilter.group;if(ga<0&&ga===b.collisionFilter.group)continue;
        const p=a.bb,q=b.bb;if(p[2]<q[0]||q[2]<p[0]||p[3]<q[1]||q[3]<p[1])continue;
        const k=collide(a,b);if(!k)continue;k.a=a;k.b=b;cts.push(k);
        const key=a.id+'_'+b.id;seen.add(key);if(!e._p.has(key))nw.push({bodyA:a,bodyB:b,collision:{supports:k.pts}})}}
    e._p=seen;if(nw.length)(e._ev.collisionStart||[]).forEach(f=>f({pairs:nw}));
    for(const k of cts){const{a,b,n}=k;k.mu=Math.sqrt(a.friction*b.friction);const rest=Math.max(a.restitution,b.restitution);
      k.P=k.pts.map(pt=>{
        const rA={x:pt.x-a.position.x,y:pt.y-a.position.y},rB={x:pt.x-b.position.x,y:pt.y-b.position.y},p={rA,rB};
        const cn=(r)=>r.x*n.y-r.y*n.x,ct=(r)=>r.x*n.x+r.y*n.y;
        p.kN=a.invMass+b.invMass+a.invInertia*cn(rA)**2+b.invInertia*cn(rB)**2;
        p.kT=a.invMass+b.invMass+a.invInertia*ct(rA)**2+b.invInertia*ct(rB)**2;
        const[vx,vy]=rv(a,b,p),vn=vx*n.x+vy*n.y;p.bn=vn<-2?-rest*vn:0;p.an=0;p.at=0;return p})}
    for(let it=0;it<8;it++){
      cs.forEach(joint);
      for(const k of cts){const{a,b,n}=k;
        for(const p of k.P){
          let[vx,vy]=rv(a,b,p),l=(p.bn-(vx*n.x+vy*n.y))/p.kN;const na=Math.max(0,p.an+l);l=na-p.an;p.an=na;imp(a,b,p.rA,p.rB,n.x*l,n.y*l);
          [vx,vy]=rv(a,b,p);const tx=-n.y,ty=n.x,mx=k.mu*p.an;let lt=-(vx*tx+vy*ty)/p.kT;const nt=cl(p.at+lt,-mx,mx);lt=nt-p.at;p.at=nt;imp(a,b,p.rA,p.rB,tx*lt,ty*lt)}}}
    for(const k of cts){const d=Math.max(0,k.depth-.5)*.4,t=k.a.invMass+k.b.invMass;if(!t)continue;
      const fa=d*k.a.invMass/t,fb=d*k.b.invMass/t;k.a.position.x-=k.n.x*fa;k.a.position.y-=k.n.y*fa;k.b.position.x+=k.n.x*fb;k.b.position.y+=k.n.y*fb}
    for(const b of bs)if(b.invMass){b.position.x+=b.velocity.x*s;b.position.y+=b.velocity.y*s;b.angle+=b.angularVelocity*s}}};

window.Matter={Engine,Bodies,Body,Composite,Constraint,Events,Vector:V,Query};
})();

/* ===== JEU ===== */
const $=s=>document.querySelector(s);
if(typeof Matter==='undefined'){$('#err').hidden=false;throw new Error('Matter.js absent')}
const {Engine,Bodies,Body,Composite,Constraint,Events,Vector}=Matter;
const C=$('#c'),X=C.getContext('2d');let W,H,DPR;
function rs(){DPR=Math.min(2,devicePixelRatio||1);W=innerWidth;H=innerHeight;C.width=W*DPR;C.height=H*DPR}
addEventListener('resize',rs);rs();

/* ---------- Réglages (localStorage) ---------- */
let S={};try{S=JSON.parse(localStorage.getItem('ps')||'{}')}catch(e){}
S.blood??=true;S.vol??=.5;S.favs??=[];S.rec??=[];
function save(){try{localStorage.setItem('ps',JSON.stringify(S))}catch(e){toast('Stockage indisponible : les réglages ne seront pas conservés.')}}
function toast(m){const t=$('#toast');t.textContent=m;t.classList.add('on');clearTimeout(toast.t);toast.t=setTimeout(()=>t.classList.remove('on'),2800)}

/* ---------- Couleurs ---------- */
const px=h=>{const n=parseInt(h.slice(1),16);return[n>>16,n>>8&255,n&255]};
function lt(h,a){const[r,g,b]=px(h),t=a<0?0:255,k=Math.abs(a);return`rgb(${r+(t-r)*k|0},${g+(t-g)*k|0},${b+(t-b)*k|0})`}
function mx(a,b,t){const A=px(a),B=px(b);return`rgb(${A.map((v,i)=>v+(B[i]-v)*t|0)})`}

/* ---------- Matériaux et catalogue ---------- */
const M={wood:{n:'Bois',c:'#b9864f',d:.0009,f:.6,r:.15},metal:{n:'Métal',c:'#9ea9b6',d:.0042,f:.35,r:.1},stone:{n:'Pierre',c:'#8b8d93',d:.0032,f:.7,r:.05},rubber:{n:'Caoutchouc',c:'#e0533d',d:.0011,f:.9,r:.85},glass:{n:'Verre',c:'#a6dcec',d:.0026,f:.15,r:.1},card:{n:'Carton',c:'#c9a577',d:.0004,f:.6,r:.05},flesh:{n:'Chair',c:'#e3b08c',d:.001,f:.5,r:.1}};
const R=(id,cat,sub,name,mat,w,h,desc,o={})=>({id,cat,sub,name,mat,w,h,desc,...o});
const PAL={skin:['#f1c9a5','#d9a27a','#a9714b','#7a4b2e','#f6d7bd'],shirt:['#d9503f','#3e7cb1','#58a55c','#e0a93b','#7b5aa6','#e8e2d6'],pants:['#2f3a52','#4a4036','#25282e','#5a6b7a']};
const rnd=a=>a[Math.random()*a.length|0];
const CAT=[
R('crate','Construction','Caisses','Caisse en bois','wood',50,50,'Caisse robuste, idéale pour les empilements.'),
R('carton','Construction','Caisses','Carton','card',56,42,'Léger et fragile, il amortit peu les chocs.'),
R('plank','Construction','Planches','Planche','wood',170,14,'Longue planche pour ponts, rampes et bascules.'),
R('beam','Construction','Poutres','Poutre en acier','metal',220,18,'Très lourde et rigide.'),
R('bar','Construction','Barres','Barre de fer','metal',140,8,'Mince, lourde, parfaite pour faire levier.'),
R('stone','Construction','Blocs','Bloc de pierre','stone',54,54,'Dense et stable, il ne rebondit presque pas.'),
R('glass','Construction','Vitres','Vitre','glass',16,100,'Se brise en éclats lors d’un impact violent.'),
R('barrel','Construction','Récipients','Tonneau','wood',38,54,'Roule mal mais se renverse volontiers.'),
R('tire','Construction','Roues','Pneu','rubber',46,46,'Rebondit et roule sans fin.',{circ:1}),
R('ball','Jouets','Balles','Balle','rubber',26,26,'Très élastique.',{circ:1}),
R('balloon','Jouets','Balles','Ballon','rubber',54,54,'Presque sans poids, le vent l’emporte.',{circ:1,dens:.00012,rest:.9,color:'#5fb8e8'}),
R('bowl','Jouets','Balles','Boule lourde','stone',32,32,'Une masse compacte qui roule.',{circ:1,color:'#34384a'}),
R('cube','Jouets','Cubes','Cube jouet','wood',30,30,'Petit cube de bois peint.',{color:'#e8564a'}),
R('table','Mobilier','Tables','Table basse','wood',110,34,'Plateau épais en bois.'),
R('bidon','Mobilier','Récipients','Bidon','metal',30,44,'Bidon métallique vide.',{color:'#c9453b'}),
R('human','Personnages','Humains','Humain','flesh',26,110,'Ragdoll articulé : clic droit pour lui donner des ordres.',{char:'human'}),
R('android','Personnages','Androïdes','Androïde','metal',26,110,'Coque métallique : les chocs provoquent des étincelles.',{char:'android'}),
R('mannequin','Personnages','Mannequins','Mannequin','wood',26,110,'Mannequin de test en bois clair.',{char:'mannequin'}),
R('plush','Personnages','Peluches','Peluche','flesh',16,66,'Petite et résistante.',{char:'plush'})];
const CF={human:()=>({skin:rnd(PAL.skin),shirt:rnd(PAL.shirt),pants:rnd(PAL.pants),hair:rnd(['#2a1b12','#6b4423','#c9a24a','#1c1c1f','#9a3b1e']),s:.9+Math.random()*.25}),
android:()=>({skin:'#aeb9c6',shirt:'#5d6e82',pants:'#3f4a59',hair:'#4fe0d2',s:1.05,android:1}),
mannequin:()=>({skin:'#d8bf94',shirt:'#d8bf94',pants:'#d8bf94',hair:'#d8bf94',s:1}),
plush:()=>({skin:'#b27c4d',shirt:'#b27c4d',pants:'#9a6a40',hair:'#7a4f2c',s:.6})};
const wt=it=>it.char?60:Math.round((it.circ?Math.PI*(it.w/2)**2:it.w*it.h)*(it.dens||M[it.mat].d)*5);

/* ---------- Monde physique ---------- */
const eng=Engine.create(),world=eng.world;
let mode='menu',tool='drag',tscale=1,lastScale=1,sel=[],drag=null,link1=null,chars=[],links=[],later=[],xray=false,follow=false,stiff=.05,pan=false;
const cam={x:0,y:-150,z:.9},G={grav:1,wind:0,hour:12},mouse={x:0,y:0};
const tw=(sx,sy)=>({x:(sx-W/2)/cam.z+cam.x,y:(sy-H/2)/cam.z+cam.y});

function buildMap(n){
  const s=(x,y,w,h,a=0,g=0)=>{const b=Bodies.rectangle(x,y,w,h,{isStatic:true,friction:.8,angle:a});b.plugin={map:1,ground:g,w,h};Composite.add(world,b)};
  s(0,60,60000,120,0,1);
  if(n==='arena'){s(-700,-250,40,620);s(700,-250,40,620);s(-300,-150,260,20);s(300,-280,260,20);s(430,-30,280,20,-.22)}
}
function resetWorld(map){Composite.clear(world,false);chars=[];links=[];sel=[];drag=null;link1=null;later=[];PT.length=0;buildMap(map);selChanged()}
const PT=[];
function burst(x,y,n,col,sp=3){for(let i=0;i<n&&PT.length<300;i++){const a=Math.random()*6.28,s=Math.random()*sp;PT.push({x,y,vx:Math.cos(a)*s,vy:Math.sin(a)*s-1,l:1,col,s:1+Math.random()*2.2})}}

function mkBody(it,x,y){
  const m=M[it.mat],o={friction:m.f,restitution:it.rest??m.r,density:it.dens||m.d,frictionAir:.005};
  const b=it.circ?Bodies.circle(x,y,it.w/2,o):Bodies.rectangle(x,y,it.w,it.h,{...o,chamfer:{radius:Math.min(it.w,it.h)*.12}});
  b.plugin={it:it.id,mat:it.mat,w:it.w,h:it.h,r:it.circ?it.w/2:0,color:it.color};return b}

/* Ragdoll : 11 segments reliés par des articulations à limites d’angle */
function ragdoll(x,y,cfg,id){
  const s=cfg.s,g=Body.nextGroup(true),col={head:cfg.skin,torso:cfg.shirt,pelvis:cfg.pants,ua:cfg.shirt,fa:cfg.skin,th:cfg.pants,sh:cfg.pants};
  const P=(k,px,py,w,h,r)=>{const o={collisionFilter:{group:g},friction:.6,restitution:.05,density:.0011,frictionAir:.01};
    const b=r?Bodies.circle(x+px*s,y+py*s,r*s,o):Bodies.rectangle(x+px*s,y+py*s,w*s,h*s,{...o,chamfer:{radius:3*s}});
    b.plugin={part:k,w:r?2*r*s:w*s,h:r?2*r*s:h*s,r:r?r*s:0,mat:cfg.android?'metal':'flesh',color:col[k],hair:cfg.hair};return b};
  const p={head:P('head',0,-30,0,0,11),torso:P('torso',0,0,26,38),pelvis:P('pelvis',0,26,24,14),
    uaL:P('ua',-16,-4,8,22),uaR:P('ua',16,-4,8,22),faL:P('fa',-16,18,7,22),faR:P('fa',16,18,7,22),
    thL:P('th',-6,45,11,28),thR:P('th',6,45,11,28),shL:P('sh',-6,73,9,28),shR:P('sh',6,73,9,28)};
  const comp=Composite.create(),J=[];
  const j=(a,pa,b,pb,lim)=>{Composite.add(comp,Constraint.create({bodyA:p[a],bodyB:p[b],pointA:{x:pa[0]*s,y:pa[1]*s},pointB:{x:pb[0]*s,y:pb[1]*s},stiffness:.85,damping:.1,length:0}));J.push({a:p[a],b:p[b],lim})};
  j('torso',[0,-19],'head',[0,11],.7);j('torso',[0,19],'pelvis',[0,-7],.5);
  j('torso',[-14,-15],'uaL',[0,-11],2.4);j('torso',[14,-15],'uaR',[0,-11],2.4);
  j('uaL',[0,11],'faL',[0,-11],2.2);j('uaR',[0,11],'faR',[0,-11],2.2);
  j('pelvis',[-6,5],'thL',[0,-14],1.2);j('pelvis',[6,5],'thR',[0,-14],1.2);
  j('thL',[0,14],'shL',[0,-14],1.8);j('thR',[0,14],'shR',[0,-14],1.8);
  Composite.add(comp,Object.values(p));Composite.add(world,comp);
  const c={comp,p,J,cfg,id,hp:100,pain:0,cons:true,alive:true,stand:false,walk:0,t:0,hr:64,o2:100,temp:37,struggle:0,shout:0,mass:Object.values(p).reduce((a,b)=>a+b.mass,0)};
  Object.values(p).forEach(b=>b.plugin.char=c);chars.push(c);return c}

function spawn(id,x,y){const it=CAT.find(i=>i.id===id);
  if(it.char){const c=ragdoll(x,y,CF[it.char](),id);c.stand=true;return c}
  const b=mkBody(it,x,y);Composite.add(world,b);return b}

function removeBody(b){
  if(b.plugin.char){const c=b.plugin.char;Composite.remove(world,c.comp);chars=chars.filter(k=>k!==c);Object.values(c.p).forEach(unlink)}
  else{Composite.remove(world,b);unlink(b)}
  if(drag&&drag.b===b||drag&&b.plugin.char&&drag.b.plugin.char===b.plugin.char)stopDrag()}
function unlink(b){links=links.filter(l=>{const dead=l.cs.some(c=>c.bodyA===b||c.bodyB===b);if(dead)l.cs.forEach(c=>Composite.remove(world,c));return!dead})}
function shatter(b){if(!Composite.allBodies(world).includes(b))return;const{x,y}=b.position;Composite.remove(world,b);unlink(b);
  for(let i=0;i<6;i++){const w=8+Math.random()*12,h=4+Math.random()*6,k=Bodies.rectangle(x+(Math.random()-.5)*20,y+(Math.random()-.5)*40,w,h,{friction:.2,restitution:.2,density:.0026,angle:Math.random()*3});
    k.plugin={mat:'glass',w,h,r:0};Body.setVelocity(k,{x:(Math.random()-.5)*5,y:-Math.random()*4});Composite.add(world,k)}
  burst(x,y,14,'#cfeff7',4);snd(1800,.25,.12,'square')}

/* ---------- Sons (Web Audio synthétisé) ---------- */
let AC,sn=0;
function snd(f,v,t=.08,ty='triangle'){if(!AC||S.vol<=0||sn>5)return;sn++;const o=AC.createOscillator(),g=AC.createGain();o.type=ty;o.frequency.value=f;g.gain.value=Math.min(.4,v)*S.vol;g.gain.exponentialRampToValueAtTime(.0001,AC.currentTime+t);o.connect(g);g.connect(AC.destination);o.start();o.stop(AC.currentTime+t)}

/* ---------- Collisions et dégâts ---------- */
Events.on(eng,'collisionStart',ev=>{
  for(const pr of ev.pairs){const A=pr.bodyA,B=pr.bodyB,v=Vector.magnitude(Vector.sub(A.velocity,B.velocity));if(v<4.5)continue;
    const pt=pr.collision.supports[0]||A.position,pa=A.plugin||{},pb=B.plugin||{};
    snd(110+Math.min(v,20)*18,v/40,.09);
    const ch=(pa.char||pb.char),mat=pa.mat==='glass'?'glass':pb.mat;
    burst(pt.x,pt.y,Math.min(8,v|0),ch?(ch.cfg.android?'#ffd36b':S.blood?'#b3202a':'#efe9d8'):mat==='glass'?'#cfeff7':mat==='metal'?'#ffe08a':'#d8c8a8');
    for(const[b,o]of[[A,B],[B,A]]){const p=b.plugin;if(!p||p.ground)continue;
      if(p.mat==='glass'&&v>8&&!b.isStatic)later.push(()=>shatter(b));
      const c=p.char;if(c&&c.alive){const k=Math.min(2.5,o.isStatic?2.2:o.mass/b.mass),d=(v-6)*k*(p.part==='head'?1.8:1);
        if(d>0){c.hp-=d;c.pain=Math.min(100,c.pain+d*1.8);if(d>12)p.fx=1}}}}});

/* ---------- Simulation d’un personnage ---------- */
const aim=(b,t,k)=>{let d=t-b.angle;d=Math.atan2(Math.sin(d),Math.cos(d));Body.setAngularVelocity(b,b.angularVelocity*.85+d*k)};
function charStep(c){
  c.pain=Math.max(0,c.pain-.05);c.t+=.016;const p=c.p;
  for(const j of c.J){let r=j.b.angle-j.a.angle;r=Math.atan2(Math.sin(r),Math.cos(r));
    if(Math.abs(r)>j.lim){Body.setAngle(j.b,j.a.angle+Math.sign(r)*j.lim);Body.setAngularVelocity(j.b,j.a.angularVelocity)}}
  if(c.alive){
    if(c.hp<=0){c.alive=false;c.cons=false;c.stand=false;c.walk=0;c.hp=0;c.hr=0}
    else{if(c.cons&&(c.hp<35||c.pain>80)){c.cons=false;c.stand=false;c.walk=0}else if(!c.cons&&c.hp>=35&&c.pain<35)c.cons=true;
      c.hr=Math.round(64+c.pain*.7+(100-c.hp)*.5);c.o2=Math.round(100-(100-c.hp)*.35);c.temp=37-(100-c.hp)*.01}
  }else{c.o2=Math.max(0,c.o2-.02);c.temp=Math.max(20,c.temp-.0005)}
  if(c.alive&&c.cons&&c.stand){
    Body.applyForce(p.pelvis,p.pelvis.position,{x:c.walk*c.mass*.00005,y:-c.mass*eng.gravity.y*.00075});
    const ph=Math.sin(c.t*9)*(c.walk?.7:0);
    aim(p.torso,0,.08);aim(p.head,0,.06);aim(p.pelvis,0,.08);aim(p.thL,ph,.1);aim(p.thR,-ph,.1);aim(p.shL,0,.1);aim(p.shR,0,.1);aim(p.uaL,0,.03);aim(p.uaR,0,.03)}
  if(c.struggle>0){c.struggle--;Body.setAngularVelocity(p[rnd(['uaL','uaR','thL','thR'])],(Math.random()-.5)*.5)}
  if(c.shout>0)c.shout--}

function step(){
  sn=0;let ts=tscale;
  if(mode==='menu'){ts=1;menuTick()}
  eng.gravity.y=G.grav;
  if(ts===0){
    if(drag){eng.gravity.scale=0;eng.timing.timeScale=1;drag.c.stiffness=.6;Engine.update(eng,16.667);
      for(const b of Composite.allBodies(world))if(!b.isStatic){Body.setVelocity(b,{x:0,y:0});Body.setAngularVelocity(b,0)}
      eng.gravity.scale=.001;drag.c.stiffness=.25}
  }else{
    eng.timing.timeScale=ts;
    if(G.wind)for(const b of Composite.allBodies(world))if(!b.isStatic)Body.applyForce(b,b.position,{x:G.wind*.00003*b.mass,y:0});
    chars.forEach(charStep);Engine.update(eng,16.667)}
  later.splice(0).forEach(f=>f());
  for(const p of PT){p.x+=p.vx*ts;p.y+=p.vy*ts;p.vy+=.12*ts;p.l-=.02*Math.max(ts,.2)}
  for(let i=PT.length;i--;)if(PT[i].l<=0)PT.splice(i,1);
  if(follow&&sel[0]){cam.x+=(sel[0].position.x-cam.x)*.1;cam.y+=(sel[0].position.y-cam.y)*.1}
  const sp=12/cam.z;if(mode==='play'&&!sel.length){if(keys.has('w')||keys.has('z')||keys.has('ArrowUp'))cam.y-=sp;if(keys.has('s')||keys.has('ArrowDown'))cam.y+=sp;if(keys.has('a')||keys.has('q')||keys.has('ArrowLeft'))cam.x-=sp;if(keys.has('d')||keys.has('ArrowRight'))cam.x+=sp}
  else if(mode==='play'){if(keys.has('ArrowUp'))cam.y-=sp;if(keys.has('ArrowDown'))cam.y+=sp;if(keys.has('ArrowLeft'))cam.x-=sp;if(keys.has('ArrowRight'))cam.x+=sp}
}

/* Fond de menu : petites scènes aléatoires */
let mt=0;
function menuTick(){cam.x=0;cam.y=-150;cam.z=Math.min(.9,W/900);
  if(--mt>0)return;mt=70;
  const ids=['crate','ball','tire','plank','balloon','cube','stone','barrel','human','android','plush','glass'],id=rnd(ids);
  spawn(id,(Math.random()-.5)*Math.min(900,W/cam.z*.8),-420);
  const bs=Composite.allBodies(world).filter(b=>!b.isStatic&&!b.plugin.part);if(bs.length>40)removeBody(bs[0])}

/* ---------- Rendu ---------- */
function rr(c,x,y,w,h,r){c.moveTo(x+r,y);c.arcTo(x+w,y,x+w,y+h,r);c.arcTo(x+w,y+h,x,y+h,r);c.arcTo(x,y+h,x,y,r);c.arcTo(x,y,x+w,y,r);c.closePath()}
const STARS=Array.from({length:70},()=>[Math.random(),Math.random()*.6,Math.random()*1.5+.4]);
function bg(){
  const d=Math.max(0,Math.sin((G.hour-6)/12*Math.PI)),g=X.createLinearGradient(0,0,0,H);
  g.addColorStop(0,mx('#070b22','#4f9ae6',d));g.addColorStop(1,mx('#1c1f40','#d3ebf8',d));X.fillStyle=g;X.fillRect(0,0,W,H);
  if(d<.4){X.fillStyle=`rgba(255,255,255,${(.4-d)*2})`;for(const[a,b,r]of STARS){X.beginPath();X.arc(a*W,b*H,r,0,7);X.fill()}}
  const day=G.hour>=6&&G.hour<18,u=day?(G.hour-6)/12:((G.hour+6)%24)/12;
  X.fillStyle=day?'#fff1b8':'#e9edf7';X.beginPath();X.arc(W*(.1+.8*u),H*.5-Math.sin(u*Math.PI)*H*.38,day?26:18,0,7);X.fill();
  const sy0=(0-cam.y)*cam.z+H/2;
  [[.15,'#7c9dbd','#1b2142',70],[.35,'#5d8a6a','#142030',40]].forEach(([f,cd,cn,a])=>{
    X.fillStyle=mx(cn,cd,d);X.beginPath();X.moveTo(0,H+5);
    for(let x=0;x<=W+8;x+=8){const wx=x/cam.z+cam.x*f;X.lineTo(x,sy0-30*cam.z-a*cam.z*(.6+.4*Math.sin(wx*.004)+.25*Math.sin(wx*.013)))}
    X.lineTo(W,H+5);X.fill()});
  const gg=X.createLinearGradient(0,sy0,0,sy0+200);gg.addColorStop(0,mx('#1a2420','#5a8f4a',d));gg.addColorStop(.08,mx('#10151a','#4a4036',d));gg.addColorStop(1,mx('#080a10','#2e2823',d));
  X.fillStyle=gg;X.fillRect(0,sy0,W,H);X.fillStyle=mx('#243a2a','#7dc05a',d);X.fillRect(0,sy0-1,W,4*cam.z)}

function texture(c,p){const w=p.w,h=p.h;c.lineWidth=1;
  if(p.mat==='wood'){c.strokeStyle='rgba(70,38,12,.35)';for(let i=1;i<5;i++){const y=-h/2+h*i/5;c.beginPath();c.moveTo(-w/2,y);c.bezierCurveTo(-w/4,y+2,w/4,y-2,w/2,y);c.stroke()}
    c.strokeStyle='rgba(40,20,5,.5)';c.beginPath();c.moveTo(w*.1,-h/2);c.lineTo(w*.14,-h*.1);c.lineTo(w*.08,h*.2);c.stroke()}
  else if(p.mat==='metal'){c.fillStyle='rgba(255,255,255,.28)';c.beginPath();c.moveTo(-w/2,-h/2);c.lineTo(-w/6,-h/2);c.lineTo(-w/2+h*.8,h/2);c.lineTo(-w/2,h/2);c.fill();
    c.strokeStyle='rgba(0,0,0,.25)';c.beginPath();c.moveTo(w*.1,-h*.2);c.lineTo(w*.3,h*.1);c.moveTo(w*.2,-h*.3);c.lineTo(w*.28,-h*.1);c.stroke()}
  else if(p.mat==='stone'){c.fillStyle='rgba(0,0,0,.18)';for(let i=0;i<14;i++)c.fillRect(((i*37)%97/97-.5)*w,((i*53)%89/89-.5)*h,2,2);c.fillStyle='rgba(255,255,255,.15)';c.fillRect(-w/2,-h/2,w,2)}
  else if(p.mat==='rubber'){c.strokeStyle='rgba(255,255,255,.55)';c.lineWidth=2.5;c.beginPath();c.arc(0,0,Math.min(w,h)*.3,3.6,4.6);c.stroke()}
  else if(p.mat==='glass'){c.fillStyle='rgba(255,255,255,.45)';c.fillRect(-w/2+w*.15,-h/2,Math.max(2,w*.12),h)}
  else if(p.mat==='card'){c.fillStyle='rgba(230,215,170,.8)';c.fillRect(-w*.08,-h/2,w*.16,h)}}

function shape(c,b,alpha){
  const p=b.plugin,m=M[p.mat]||M.wood,col=p.color||m.c;
  c.save();c.translate(b.position.x,b.position.y);c.rotate(b.angle);c.globalAlpha=alpha*(p.mat==='glass'?.65:1);
  c.beginPath();if(p.r)c.arc(0,0,p.r,0,7);else rr(c,-p.w/2,-p.h/2,p.w,p.h,Math.min(p.w,p.h)*.14);
  const g=c.createLinearGradient(-p.w/2,-p.h/2,p.w/2,p.h/2);g.addColorStop(0,lt(col,.25));g.addColorStop(1,lt(col,-.25));c.fillStyle=g;c.fill();
  c.save();c.clip();if(!p.part)texture(c,p);c.restore();
  c.lineWidth=p.frozen?2.6:1.5;c.strokeStyle=p.frozen?'#7fe3ff':'rgba(10,12,18,.85)';c.stroke();
  if(p.part==='head'){const ch=p.char;c.fillStyle=p.hair;c.beginPath();c.arc(0,0,p.r+.5,Math.PI,2*Math.PI);c.fill();
    c.strokeStyle='#1a1410';c.fillStyle='#1a1410';c.lineWidth=1.4;
    if(ch.alive&&ch.cons){c.beginPath();c.arc(-4,1,1.5,0,7);c.arc(4,1,1.5,0,7);c.fill();c.beginPath();c.arc(0,4,3,.2,2.9);c.stroke()}
    else{c.beginPath();for(const x of[-4,4]){c.moveTo(x-2,-1);c.lineTo(x+2,3);c.moveTo(x+2,-1);c.lineTo(x-2,3)}c.stroke()}}
  if(p.part==='sh'){c.fillStyle='rgba(0,0,0,.45)';c.fillRect(-p.w/2,p.h/2-5,p.w,5)}
  c.restore()}

function bones(c,b){const p=b.plugin;c.save();c.translate(b.position.x,b.position.y);c.rotate(b.angle);c.strokeStyle=p.fx?'#ff5a5a':'#f4f1e6';c.lineWidth=2.4;c.lineCap='round';c.beginPath();
  if(p.part==='head'){c.arc(0,-1,p.r*.75,0,7);c.moveTo(-p.r*.4,p.r*.55);c.lineTo(p.r*.4,p.r*.55)}
  else if(p.part==='torso'){c.moveTo(0,-p.h/2);c.lineTo(0,p.h/2);for(let i=0;i<4;i++){const y=-p.h*.3+i*p.h*.14;for(const s of[-1,1]){c.moveTo(0,y);c.quadraticCurveTo(s*p.w*.5,y-3,s*p.w*.38,y+5)}}}
  else if(p.part==='pelvis'){c.ellipse(0,0,p.w*.38,p.h*.35,0,0,7)}
  else{c.moveTo(0,-p.h*.45);c.lineTo(0,p.h*.45)}
  c.stroke();c.restore()}

function ep(c){const f=(b,o)=>b?Vector.add(b.position,Vector.rotate(o,b.angle)):o;return[f(c.bodyA,c.pointA),f(c.bodyB,c.pointB)]}
function drawLink(l){X.lineWidth=3;X.lineCap='round';
  for(const c of l.cs){const[a,b]=ep(c);
    if(l.t==='spring'){const n=10,dx=b.x-a.x,dy=b.y-a.y,L=Math.hypot(dx,dy)||1,nx=-dy/L*7,ny=dx/L*7;X.strokeStyle='#ffd27a';X.beginPath();X.moveTo(a.x,a.y);for(let i=1;i<n;i++){const s=i%2?1:-1;X.lineTo(a.x+dx*i/n+nx*s,a.y+dy*i/n+ny*s)}X.lineTo(b.x,b.y);X.stroke()}
    else if(l.t==='rope'){X.strokeStyle='#c8a46a';X.setLineDash([6,3]);X.beginPath();X.moveTo(a.x,a.y);X.lineTo(b.x,b.y);X.stroke();X.setLineDash([])}
    else if(l.t==='weld'&&c===l.cs[0]){X.fillStyle='#ffb454';for(const q of[a,b]){X.beginPath();X.arc(q.x,q.y,4,0,7);X.fill()}}
    else if(l.t==='pin'){X.fillStyle='#ff6a5a';X.strokeStyle='#220';X.lineWidth=1;X.beginPath();X.arc(b.x,b.y,5,0,7);X.fill();X.stroke()}}}

function render(){
  X.setTransform(DPR,0,0,DPR,0,0);bg();
  X.setTransform(DPR*cam.z,0,0,DPR*cam.z,DPR*(W/2-cam.x*cam.z),DPR*(H/2-cam.y*cam.z));
  const bs=Composite.allBodies(world);
  for(const b of bs){const p=b.plugin;if(p.map&&!p.ground){X.save();X.translate(b.position.x,b.position.y);X.rotate(b.angle);X.fillStyle='#3a414d';X.fillRect(-p.w/2,-p.h/2,p.w,p.h);X.fillStyle='#5a6474';X.fillRect(-p.w/2,-p.h/2,p.w,4);X.restore()}}
  links.forEach(drawLink);
  for(const b of bs){const p=b.plugin;if(p.map)continue;shape(X,b,xray&&p.part?.22:1);if(xray&&p.part)bones(X,b)}
  X.fillStyle='#fff';X.font='800 18px sans-serif';
  for(const c of chars)if(c.shout>0)X.fillText('AAAH !',c.p.head.position.x-24,c.p.head.position.y-26);
  X.strokeStyle='#ffb454';X.lineWidth=2;X.setLineDash([5,4]);
  for(const b of sel){const p=b.plugin;X.save();X.translate(b.position.x,b.position.y);X.rotate(b.angle);X.beginPath();if(p.r)X.arc(0,0,p.r+3,0,7);else X.rect(-p.w/2-3,-p.h/2-3,p.w+6,p.h+6);X.stroke();X.restore()}
  X.setLineDash([]);
  for(const p of PT){X.globalAlpha=Math.max(0,p.l);X.fillStyle=p.col;X.beginPath();X.arc(p.x,p.y,p.s,0,7);X.fill()}X.globalAlpha=1;
  if(drag){const a=drag.c.pointA,[,q]=ep(drag.c);X.strokeStyle='rgba(255,180,84,.8)';X.lineWidth=2;X.beginPath();X.moveTo(a.x,a.y);X.lineTo(q.x,q.y);X.stroke()}
  if(link1){const q=tw(mouse.x,mouse.y),a=link1.world();X.strokeStyle='#5fd3c4';X.lineWidth=2;X.setLineDash([4,4]);X.beginPath();X.moveTo(a.x,a.y);X.lineTo(q.x,q.y);X.stroke();X.setLineDash([])}
  X.setTransform(DPR,0,0,DPR,0,0);const d=Math.max(0,Math.sin((G.hour-6)/12*Math.PI));
  if(d<1){X.fillStyle=`rgba(4,6,28,${(1-d)*.4})`;X.fillRect(0,0,W,H)}
}

/* ---------- Sélection, outils, saisie ---------- */
const pick=pt=>{const l=Matter.Query.point(Composite.allBodies(world).filter(b=>!b.plugin.ground),pt);return l[l.length-1]};
const rootOf=b=>b&&(b.plugin.char||b);
function stopDrag(){if(drag){Composite.remove(world,drag.c);drag=null}}
const loc=(b,pt)=>Vector.rotate(Vector.sub(pt,b.position),-b.angle);
function makeLink(t,a,pa,b,pb){
  const wa=Vector.add(a.position,Vector.rotate(pa,a.angle)),wb=Vector.add(b.position,Vector.rotate(pb,b.angle)),d=Math.max(4,Vector.magnitude(Vector.sub(wb,wa)));
  const mk=o=>Constraint.create({bodyA:a,bodyB:b,...o});let cs;
  if(t==='weld')cs=[mk({pointA:pa,pointB:pb,stiffness:1,length:d}),mk({stiffness:1,length:Vector.magnitude(Vector.sub(b.position,a.position))})];
  else if(t==='rope')cs=[mk({pointA:pa,pointB:pb,stiffness:Math.max(.2,stiff*4),damping:.1,length:d})];
  else cs=[mk({pointA:pa,pointB:pb,stiffness:stiff*.6,damping:.02,length:d})];
  Composite.add(world,cs);links.push({t,cs})}

C.addEventListener('pointerdown',e=>{
  if(!AC)try{AC=new(window.AudioContext||window.webkitAudioContext)()}catch(_){}
  C.setPointerCapture(e.pointerId);hideCtx();mouse.x=e.clientX;mouse.y=e.clientY;
  if(e.button===1){pan=true;return}if(e.button!==0||mode!=='play')return;
  const w=tw(e.clientX,e.clientY),b=pick(w);
  if(tool==='drag'){
    if(b&&!b.plugin.map){if(e.shiftKey){sel.includes(b)?sel.splice(sel.indexOf(b),1):sel.push(b)}else sel=[b];
      if(!b.isStatic){drag={b,c:Constraint.create({pointA:w,bodyB:b,pointB:loc(b,w),stiffness:.25,damping:.12,length:0})};Composite.add(world,drag.c);snd(300,.1,.05)}}
    else if(!e.shiftKey)sel=[];selChanged()}
  else if(tool==='freeze'){if(b&&!b.plugin.map){Body.setStatic(b,!b.isStatic);b.plugin.frozen=b.isStatic;snd(900,.1,.1,'sine');selChanged()}}
  else if(tool==='erase'){
    const hit=links.find(l=>l.cs.some(c=>{const[a,q]=ep(c),t=Math.max(0,Math.min(1,((w.x-a.x)*(q.x-a.x)+(w.y-a.y)*(q.y-a.y))/((q.x-a.x)**2+(q.y-a.y)**2||1)));return Math.hypot(a.x+(q.x-a.x)*t-w.x,a.y+(q.y-a.y)*t-w.y)<8/cam.z}));
    if(hit){hit.cs.forEach(c=>Composite.remove(world,c));links=links.filter(l=>l!==hit);snd(500,.15,.1)}
    else if(b&&!b.plugin.map){sel=sel.filter(s=>rootOf(s)!==rootOf(b));removeBody(b);snd(220,.2,.12);selChanged()}}
  else if(tool==='pin'){if(b){const o=loc(b,w),c=Constraint.create({pointA:w,bodyB:b,pointB:o,stiffness:1,length:0});Composite.add(world,c);links.push({t:'pin',cs:[c]});snd(700,.15,.06)}else toast('Cliquez sur un objet pour l’épingler.')}
  else{ /* soudure, corde, ressort */
    if(!b){link1=null;toast('Cliquez sur deux objets pour les relier.');return}
    if(!link1){link1={b,o:loc(b,w),world(){return Vector.add(this.b.position,Vector.rotate(this.o,this.b.angle))}};snd(600,.1,.05)}
    else if(link1.b===b){toast('Choisissez un second objet différent.')}
    else{makeLink(tool,link1.b,link1.o,b,loc(b,w));link1=null;snd(450,.15,.08)}}
});
C.addEventListener('pointermove',e=>{
  if(pan){cam.x-=(e.clientX-mouse.x)/cam.z;cam.y-=(e.clientY-mouse.y)/cam.z}
  mouse.x=e.clientX;mouse.y=e.clientY;if(drag)drag.c.pointA=tw(e.clientX,e.clientY)});
C.addEventListener('pointerup',()=>{pan=false;stopDrag()});
C.addEventListener('wheel',e=>{e.preventDefault();const a=tw(e.clientX,e.clientY);cam.z=Math.max(.25,Math.min(3,cam.z*(e.deltaY<0?1.1:.9)));const b=tw(e.clientX,e.clientY);cam.x+=a.x-b.x;cam.y+=a.y-b.y},{passive:false});

/* Menu contextuel d’un personnage (clic droit) */
function hideCtx(){$('#ctx').style.display='none'}
C.addEventListener('contextmenu',e=>{e.preventDefault();if(mode!=='play')return;
  const b=pick(tw(e.clientX,e.clientY)),c=b&&b.plugin.char;if(!c)return;
  const A=[['Se lever',()=>{if(c.alive&&c.cons){c.stand=true;c.walk=0}else toast('Ce personnage ne peut pas se lever.')}],
   ['Marcher à droite',()=>{if(c.alive&&c.cons){c.stand=true;c.walk=1}}],['Marcher à gauche',()=>{if(c.alive&&c.cons){c.stand=true;c.walk=-1}}],
   ['S’allonger',()=>{c.stand=false;c.walk=0}],['Se débattre',()=>{if(c.alive)c.struggle=120}],
   ['Crier',()=>{if(c.alive&&c.cons){c.shout=70;snd(520,.3,.4,'sawtooth')}}],
   ['Réveiller',()=>{if(c.alive&&c.hp>=20){c.cons=true;c.pain=0}else toast('Impossible de réveiller ce personnage.')}]];
  const m=$('#ctx');m.innerHTML='';A.forEach(([n,f])=>{const k=document.createElement('button');k.textContent=n;k.onclick=()=>{f();hideCtx()};m.appendChild(k)});
  m.style.display='flex';m.style.left=Math.min(e.clientX,W-170)+'px';m.style.top=Math.min(e.clientY,H-290)+'px'});

/* Clavier */
const keys=new Set();
addEventListener('keydown',e=>{if(e.target.tagName==='INPUT'&&e.target.type!=='range')return;const k=e.key.length===1?e.key.toLowerCase():e.key;keys.add(k);if(mode!=='play')return;
  if(k===' '){e.preventDefault();setScale(tscale?0:(lastScale||1))}
  else if(k==='Delete'||k==='Backspace'){const r=[...new Set(sel.map(rootOf))];r.forEach(x=>removeBody(x.p?x.p.torso:x));sel=[];selChanged()}
  else if(e.ctrlKey&&k==='d'){e.preventDefault();dupe()}
  else if(k==='a'||k==='e'){const d=k==='a'?-.15:.15;for(const x of new Set(sel.map(rootOf)))for(const b of(x.p?Object.values(x.p):[x]))Body.setAngle(b,b.angle+d)}});
addEventListener('keyup',e=>keys.delete(e.key.length===1?e.key.toLowerCase():e.key));
function dupe(){const n=[];for(const x of new Set(sel.map(rootOf))){
  if(x.p){const t=x.p.torso.position,c=ragdoll(t.x+70,t.y-20,{...x.cfg},x.id);c.stand=x.stand;n.push(c.p.torso)}
  else if(x.plugin.it){const it=CAT.find(i=>i.id===x.plugin.it),b=mkBody(it,x.position.x+40,x.position.y-40);Body.setAngle(b,x.angle);Composite.add(world,b);n.push(b)}}
  sel=n;selChanged()}

/* ---------- Interface ---------- */
const TOOLS=[['drag','Glisser','M12 3v18M3 12h18M12 3l-3 3M12 3l3 3M12 21l-3-3M12 21l3-3M3 12l3-3M3 12l3 3M21 12l-3-3M21 12l-3 3'],
['freeze','Figer','M12 3v18M4.5 7.5l15 9M19.5 7.5l-15 9'],['pin','Épingle','M12 4a4 4 0 100 8 4 4 0 000-8zM12 12v9'],
['weld','Soudure','M4 12h6M14 12h6M10 8v8M14 8v8'],['rope','Corde','M4 18c4-10 6 4 8-4s6-8 8-6'],
['spring','Ressort','M3 12h3l2-5 3 10 3-10 3 10 2-5h3'],['erase','Gomme','M5 15l8-9 6 6-6 7H8zM9 19h11']];
TOOLS.forEach(([id,n,d])=>{const b=document.createElement('button');b.title=n;b.setAttribute('aria-label',n);b.dataset.t=id;b.innerHTML=`<svg viewBox="0 0 24 24"><path d="${d}"/></svg>`;
  b.onclick=()=>{tool=id;link1=null;C.style.cursor=id==='drag'?'grab':id==='erase'?'not-allowed':'crosshair';[...$('#tools').children].forEach(x=>x.classList.toggle('on',x===b));snd(500,.08,.05)};$('#tools').appendChild(b)});
$('#tools').firstChild.classList.add('on');
[['100 %',1],['50 %',.5],['20 %',.2],['0 %',0]].forEach(([n,v])=>{const b=document.createElement('button');b.textContent=n;b.dataset.v=v;b.onclick=()=>setScale(v);$('#time').appendChild(b)});
function setScale(v){if(tscale)lastScale=tscale;tscale=v;[...$('#time').children].forEach(b=>b.classList.toggle('on',+b.dataset.v===v))}
setScale(1);

let cat='Tous';
function icon(it){const c=document.createElement('canvas');c.width=c.height=44;const k=c.getContext('2d');k.translate(22,22);
  if(it.char){const cf=CF[it.char]();const t=it.char==='plush'?.7:1;k.scale(t,t);k.fillStyle=cf.shirt;k.beginPath();rr(k,-9,-4,18,24,5);k.fill();k.fillStyle=cf.skin;k.beginPath();k.arc(0,-11,9,0,7);k.fill();k.fillStyle=cf.hair;k.beginPath();k.arc(0,-11,9,Math.PI,2*Math.PI);k.fill();k.strokeStyle='#111';k.lineWidth=1.3;k.beginPath();k.arc(0,-11,9,0,7);rr(k,-9,-4,18,24,5);k.stroke()}
  else{const s=Math.min(36/it.w,36/it.h,1.3);k.scale(s,s);shape(k,{position:{x:0,y:0},angle:0,plugin:{mat:it.mat,w:it.w,h:it.h,r:it.circ?it.w/2:0,color:it.color}},1)}return c}
function renderList(){
  const q=$('#q').value.trim().toLowerCase(),sort=$('#sort').value;
  let L=CAT.filter(i=>(cat==='Tous'||(cat==='Favoris'?S.favs.includes(i.id):cat==='Récents'?S.rec.includes(i.id):i.cat===cat))&&(!q||(i.name+' '+i.sub+' '+M[i.mat].n+' '+i.cat).toLowerCase().includes(q)));
  L.sort((a,b)=>sort==='w'?wt(a)-wt(b):sort==='mat'?M[a.mat].n.localeCompare(M[b.mat].n):sort==='name'?a.name.localeCompare(b.name):a.cat.localeCompare(b.cat)||a.sub.localeCompare(b.sub));
  const list=$('#list');list.innerHTML='';if(!L.length)list.textContent='Aucun objet trouvé. Modifiez la recherche ou la catégorie.';
  for(const it of L){const d=document.createElement('div');d.className='it';d.appendChild(icon(it));d.append(it.name);
    const f=document.createElement('b');f.textContent='★';f.title='Favori';f.className=S.favs.includes(it.id)?'f':'';f.onclick=e=>{e.stopPropagation();S.favs=S.favs.includes(it.id)?S.favs.filter(x=>x!==it.id):[...S.favs,it.id];save();renderList()};d.appendChild(f);
    d.onmouseenter=()=>{$('#info').textContent=`${it.name} · ${it.sub} · ${M[it.mat].n} · env. ${wt(it)} kg. ${it.desc}`};
    d.onclick=()=>{const p=tw(W/2,H/3),o=spawn(it.id,p.x+(Math.random()-.5)*120,p.y);sel=[o.p?o.p.torso:o];selChanged();S.rec=[it.id,...S.rec.filter(x=>x!==it.id)].slice(0,8);save();snd(400,.12,.06)};
    list.appendChild(d)}}
['Tous','Favoris','Récents',...new Set(CAT.map(i=>i.cat))].forEach(n=>{const b=document.createElement('button');b.textContent=n;b.onclick=()=>{cat=n;[...$('#chips').children].forEach(x=>x.classList.toggle('on',x===b));renderList()};if(n==='Tous')b.className='on';$('#chips').appendChild(b)});
$('#q').oninput=renderList;$('#sort').onchange=renderList;renderList();

function selChanged(){const b=sel[0],c=b&&b.plugin.char;$('#props').hidden=!b;$('#stat').hidden=!c;if(!b)return;
  $('#pm').value=Math.round(b.mass*5);$('#pf').value=b.friction;$('#pe').value=b.restitution;$('#pc').value=b.plugin.color||(M[b.plugin.mat]||M.wood).c;
  $('#pst').textContent=b.isStatic?'État : figé':'État : libre'}
$('#pm').oninput=e=>sel.forEach(b=>Body.setMass(b,e.target.value/5));
$('#pf').oninput=e=>sel.forEach(b=>b.friction=+e.target.value);
$('#pe').oninput=e=>sel.forEach(b=>b.restitution=+e.target.value);
$('#pc').oninput=e=>sel.forEach(b=>b.plugin.color=e.target.value);
$('#wg').oninput=e=>G.grav=+e.target.value;$('#ww').oninput=e=>G.wind=+e.target.value;$('#wh').oninput=e=>G.hour=+e.target.value;$('#wk').oninput=e=>stiff=+e.target.value;
$('#bXray').onclick=e=>{xray=!xray;e.target.classList.toggle('on',xray)};
$('#bFollow').onclick=e=>{follow=!follow;e.target.classList.toggle('on',follow);if(follow&&!sel.length)toast('Sélectionnez un objet ou un personnage à suivre.')};
let clr=0;$('#bClear').onclick=e=>{if(!clr){e.target.textContent='Confirmer ?';clr=setTimeout(()=>{clr=0;e.target.textContent='Tout supprimer'},3000)}else{clearTimeout(clr);clr=0;e.target.textContent='Tout supprimer';resetWorld(curMap);toast('Scène vidée.')}};
$('#bMenu').onclick=()=>{goMenu()};

/* ECG et état du personnage */
const ecg=[];let phase=0;
function hud(){const c=sel[0]&&sel[0].plugin.char;if(!c||$('#stat').hidden)return;
  $('#sHp').textContent=Math.round(c.hp)+' %';$('#bHp').style.width=c.hp+'%';$('#sPain').textContent=Math.round(c.pain);$('#bPain').style.width=c.pain+'%';
  $('#sCons').textContent=c.alive?(c.cons?'Conscient':'Inconscient'):'Aucune';
  $('#sSt').textContent=!c.alive?'Arrêt cardiaque':!c.cons?'Au sol, inconscient':c.stand?(c.walk?'Marche':'Debout'):'Au sol';
  $('#sHr').textContent=c.hr+' bpm';$('#sO2').textContent=Math.round(c.o2)+' %';$('#sTp').textContent=c.temp.toFixed(1)+' °C';
  phase+=c.hr/60/60;const p=phase%1;let v=0;
  if(c.hr){if(p<.08)v=.15*Math.sin(p/.08*Math.PI);else if(p>.16&&p<.2)v=-.15;else if(p>=.2&&p<.26)v=Math.sin((p-.2)/.06*Math.PI);else if(p>=.26&&p<.3)v=-.25*Math.sin((p-.26)/.04*Math.PI);else if(p>.4&&p<.55)v=.25*Math.sin((p-.4)/.15*Math.PI)}
  ecg.push(v);if(ecg.length>200)ecg.shift();
  const k=$('#ecg').getContext('2d');k.clearRect(0,0,200,44);k.strokeStyle='#5fd3c4';k.lineWidth=1.6;k.beginPath();ecg.forEach((y,i)=>i?k.lineTo(i,34-y*28):k.moveTo(i,34-y*28));k.stroke()}

/* Menu principal */
const LOGO=$('#logo');['PEOPLE','SANDBOX'].forEach(w=>{const d=document.createElement('div');[...w].forEach((ch,i)=>{const s=document.createElement('span');s.textContent=ch;s.style.animationDelay=(i*.15)+'s';d.appendChild(s)});LOGO.appendChild(d)});
function sec(id){['main','play','set','cred'].forEach(n=>$('#m-'+n).hidden=n!==id)}
document.querySelectorAll('[data-go]').forEach(b=>b.onclick=()=>sec(b.dataset.go));
let curMap='plain';
document.querySelectorAll('[data-map]').forEach(b=>b.onclick=()=>{curMap=b.dataset.map;resetWorld(curMap);mode='play';$('#menu').hidden=true;$('#hud').hidden=false;
  cam.x=0;cam.y=-120;cam.z=1;setScale(1);spawn('human',0,-100);toast('Clic droit sur un personnage pour lui donner des ordres.')});
function goMenu(){mode='menu';resetWorld('plain');$('#menu').hidden=false;$('#hud').hidden=true;sec('main');setScale(1);follow=false;$('#bFollow').classList.remove('on')}
$('#sBlood').checked=S.blood;$('#sBlood').onchange=e=>{S.blood=e.target.checked;save()};
$('#sVol').value=S.vol;$('#sVol').oninput=e=>{S.vol=+e.target.value;save()};

resetWorld('plain');
(function loop(){step();render();hud();requestAnimationFrame(loop)})();
