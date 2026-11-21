<!DOCTYPE html>
<html lang="fr"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>People Sandbox</title>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Bricolage+Grotesque:wght@500;800&display=swap">
<style>
:root{--glass:rgba(20,24,34,.74);--line:rgba(255,255,255,.13);--tx:#eceff5;--mute:#9aa4b6;--acc:#ffb454;--acc2:#5fd3c4;color-scheme:dark;box-sizing:border-box;padding-top:env(safe-area-inset-top,0px);padding-bottom:env(safe-area-inset-bottom,0px)}
*{box-sizing:border-box}
html,body{height:100%;margin:0;overflow:hidden;background:#0d1020;color:var(--tx);font:500 13px/1.35 'Bricolage Grotesque',system-ui,sans-serif;user-select:none}
canvas#c{position:fixed;inset:0;display:block;touch-action:none}
.glass{background:var(--glass);backdrop-filter:blur(14px);-webkit-backdrop-filter:blur(14px);border:1px solid var(--line);border-radius:14px}
button,select,input{font:inherit;color:inherit}
button{background:rgba(255,255,255,.07);border:1px solid var(--line);border-radius:9px;padding:6px 10px;cursor:pointer}
button:hover{background:rgba(255,255,255,.14)}button.on{background:var(--acc);color:#1a1405;border-color:var(--acc)}
button:focus-visible,input:focus-visible,select:focus-visible{outline:2px solid var(--acc2);outline-offset:1px}
input[type=text],input:not([type]),select{background:rgba(0,0,0,.3);border:1px solid var(--line);border-radius:8px;padding:6px 8px;width:100%}
#bar{position:fixed;left:10px;top:calc(10px + env(safe-area-inset-top,0px));bottom:70px;width:224px;padding:10px;display:flex;flex-direction:column;gap:7px}
#chips{display:flex;flex-wrap:wrap;gap:4px}#chips button{padding:3px 8px;font-size:12px}
#list{flex:1;overflow:auto;display:grid;grid-template-columns:1fr 1fr;gap:6px;align-content:start}
.it{position:relative;background:rgba(255,255,255,.05);border:1px solid var(--line);border-radius:10px;padding:6px 4px;text-align:center;cursor:pointer;font-size:12px}
.it:hover{background:rgba(255,255,255,.12)}.it canvas{display:block;margin:0 auto 3px}
.it b{position:absolute;right:4px;top:1px;color:var(--mute);font-size:14px;cursor:pointer}.it b.f{color:var(--acc)}
#info{font-size:12px;color:var(--mute);min-height:48px}
#tools{position:fixed;bottom:calc(10px + env(safe-area-inset-bottom,0px));left:50%;transform:translateX(-50%);display:flex;gap:4px;padding:6px}
#tools button{width:46px;height:46px;padding:0;display:grid;place-items:center}
#tools svg{width:24px;height:24px;stroke:currentColor;fill:none;stroke-width:1.8;stroke-linecap:round;stroke-linejoin:round}
#time{position:fixed;bottom:calc(10px + env(safe-area-inset-bottom,0px));left:10px;display:flex;gap:4px;padding:6px}
#panel{position:fixed;right:10px;top:calc(10px + env(safe-area-inset-top,0px));width:232px;max-height:calc(100% - 80px);overflow:auto;padding:10px;display:flex;flex-direction:column;gap:8px}
#panel h4{margin:4px 0 0;font-size:13px;color:var(--acc)}
.row{display:flex;gap:4px;flex-wrap:wrap}.row button{flex:1}
label{display:grid;grid-template-columns:78px 1fr;align-items:center;gap:6px;font-size:12px;color:var(--mute)}
label input[type=range]{width:100%}
.bar{height:6px;border-radius:3px;background:rgba(255,255,255,.12);overflow:hidden}.bar i{display:block;height:100%;background:var(--acc2)}
#stat div.k{display:flex;justify-content:space-between;font-size:12px}
#toast{position:fixed;top:14px;left:50%;transform:translate(-50%,-80px);background:var(--glass);border:1px solid var(--acc);padding:8px 14px;border-radius:10px;transition:.25s;z-index:9}
#toast.on{transform:translate(-50%,0)}
#ctx{position:fixed;display:none;flex-direction:column;gap:3px;padding:6px;z-index:8}#ctx button{text-align:left}
#menu{position:fixed;inset:0;display:grid;place-items:center;background:radial-gradient(ellipse at center,rgba(8,10,22,.15),rgba(8,10,22,.7));z-index:5}
.mwrap{text-align:center;display:flex;flex-direction:column;align-items:center;gap:14px;max-width:min(92vw,520px)}
.logo{margin:0;font:800 clamp(44px,10vw,92px)/.9 'Bricolage Grotesque',sans-serif;letter-spacing:-.02em}
.logo div{display:flex;justify-content:center}.logo span{display:inline-block;animation:bob 2.4s ease-in-out infinite;color:#f4efe3}
.logo div+div span{color:var(--acc)}
@keyframes bob{0%,100%{transform:translateY(0) rotate(-2deg)}50%{transform:translateY(-8px) rotate(2deg)}}
.mwrap nav{display:flex;flex-direction:column;gap:8px;width:230px}.mwrap nav button{padding:11px;font-size:16px;font-weight:800}
.sec{background:var(--glass);border:1px solid var(--line);border-radius:16px;padding:22px;backdrop-filter:blur(14px)}
.sec h2{margin:0}.sec p{margin:0;color:var(--mute)}
#err{position:fixed;inset:0;display:grid;place-items:center;background:#0d1020;z-index:20;text-align:center;padding:20px}
@media (prefers-reduced-motion:reduce){.logo span{animation:none}}
@media (max-width:700px){#bar{width:150px}#panel{width:190px}#tools button{width:38px;height:38px}}
</style></head><body>
<canvas id="c"></canvas>
<div id="err" hidden><div><h2>Moteur physique introuvable</h2><p>Le fichier app.js est introuvable. Placez-le dans le même dossier que index.html.</p></div></div>
<div id="toast"></div><div id="ctx" class="glass"></div>

<div id="hud" hidden>
 <aside id="bar" class="glass">
  <input id="q" placeholder="Rechercher un objet" autocomplete="off">
  <select id="sort"><option value="cat">Trier par type</option><option value="name">Trier par nom</option><option value="mat">Trier par matériau</option><option value="w">Trier par poids</option></select>
  <div id="chips"></div><div id="list"></div><div id="info">Cliquez sur un objet pour le déposer. Survolez-le pour lire sa fiche.</div>
 </aside>
 <div id="tools" class="glass"></div>
 <div id="time" class="glass"></div>
 <aside id="panel" class="glass">
  <div class="row"><button id="bMenu">Menu</button><button id="bXray">Radiographie</button><button id="bFollow">Suivre</button><button id="bClear">Tout supprimer</button></div>
  <div id="props" hidden><h4>Propriétés</h4>
   <label>Masse (kg)<input id="pm" type="range" min="1" max="300" step="1"></label>
   <label>Friction<input id="pf" type="range" min="0" max="1.2" step=".05"></label>
   <label>Élasticité<input id="pe" type="range" min="0" max="1" step=".05"></label>
   <label>Couleur<input id="pc" type="color"></label>
   <div id="pst" style="color:var(--mute)"></div></div>
  <div id="stat" hidden><h4>État du personnage</h4>
   <div class="k"><span>Santé</span><span id="sHp"></span></div><div class="bar"><i id="bHp"></i></div>
   <div class="k"><span>Douleur</span><span id="sPain"></span></div><div class="bar"><i id="bPain" style="background:#e5675a"></i></div>
   <div class="k"><span>Conscience</span><span id="sCons"></span></div>
   <div class="k"><span>État physique</span><span id="sSt"></span></div>
   <div class="k"><span>Rythme cardiaque</span><span id="sHr"></span></div>
   <canvas id="ecg" width="200" height="44" style="width:100%;background:rgba(0,0,0,.35);border-radius:8px"></canvas>
   <div class="k"><span>Oxygène</span><span id="sO2"></span></div>
   <div class="k"><span>Température</span><span id="sTp"></span></div></div>
  <h4>Monde</h4>
  <label>Gravité<input id="wg" type="range" min="0" max="2" step=".05" value="1"></label>
  <label>Vent<input id="ww" type="range" min="-1" max="1" step=".05" value="0"></label>
  <label>Heure<input id="wh" type="range" min="0" max="24" step=".25" value="12"></label>
  <label>Raideur liens<input id="wk" type="range" min=".01" max=".9" step=".01" value=".05"></label>
 </aside>
</div>

<div id="menu">
 <div class="mwrap" id="m-main"><h1 class="logo" id="logo"></h1>
  <nav><button data-go="play">Jouer</button><button data-go="set">Paramètres</button><button data-go="cred">Crédits</button></nav></div>
 <div class="mwrap sec" id="m-play" hidden><h2>Choisir une carte</h2>
  <button data-map="plain" style="width:100%;padding:12px"><b>Plaine</b><br><span style="color:var(--mute)">Grand terrain plat pour tout tester.</span></button>
  <button data-map="arena" style="width:100%;padding:12px"><b>Arène</b><br><span style="color:var(--mute)">Murs, plateformes et rampe.</span></button>
  <button data-go="main">Retour</button></div>
 <div class="mwrap sec" id="m-set" hidden><h2>Paramètres</h2>
  <label style="width:100%">Sang et blessures<input id="sBlood" type="checkbox" style="justify-self:start"></label>
  <label style="width:100%">Volume<input id="sVol" type="range" min="0" max="1" step=".05"></label>
  <button data-go="main">Retour</button></div>
 <div class="mwrap sec" id="m-cred" hidden><h2>Crédits</h2>
  <p>Physique : moteur maison intégré à app.js.<br>Police : Bricolage Grotesque (licence SIL OFL).<br>Graphismes dessinés par le code, sons synthétisés par Web Audio : tout est original.</p>
  <button data-go="main">Retour</button></div>
</div>

<script src="app.js"></script>
</body></html>
