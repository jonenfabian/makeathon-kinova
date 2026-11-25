// Kinova-Dashboard: Bedienung und Digital Twin (three.js, z nach oben wie die Roboterbasis).
import * as THREE from 'three';
import { OrbitControls } from '/vendor/three/OrbitControls.js';
import { STLLoader } from '/vendor/three/STLLoader.js';

const $ = (id) => document.getElementById(id);
const log = (text) => { const l = $('log'); l.textContent = `${new Date().toLocaleTimeString()}  ${text}\n` + l.textContent; };

// Betreuer am Mac Mini: Kopfzeile X-Kinova. Team-Laptop: Team-Schlüssel und Fahrrecht (siehe netz.py).
const lesen = (speicher, name) => { try { return speicher.getItem(name); } catch { return null; } };
const merken = (speicher, name, wert) => { try { if (wert) speicher.setItem(name, wert); else speicher.removeItem(name); } catch { /* ohne Speicher */ } };
const team = { schluessel: lesen(localStorage, 'kinova_schluessel'), name: lesen(localStorage, 'kinova_team'),
  fahrrecht: lesen(sessionStorage, 'kinova_fahrrecht') };
function kopf() {
  const k = { 'Content-Type': 'application/json' };
  if (!team.schluessel) return { ...k, 'X-Kinova': '1' };
  return { ...k, 'X-Kinova-Schluessel': team.schluessel, ...(team.fahrrecht ? { 'X-Kinova-Fahrrecht': team.fahrrecht } : {}) };
}
async function post(url, body = {}) {
  const antwort = await fetch(url, { method: 'POST', headers: kopf(), body: JSON.stringify(body) });
  const daten = await antwort.json().catch(() => ({}));
  if (!antwort.ok) { log(`✗ ${url}: ${daten.fehler || antwort.status}`); throw new Error(daten.fehler || antwort.status); }
  log(`✓ ${url}: ${kurz(daten)}`);
  return daten;
}
const kurz = (d) => d.meldung || (d.verlauf ? `${d.verlauf.length} Schritte, zuletzt: ${d.verlauf.at(-1)?.schritt ?? '–'}` : null) || (d.schritte ? d.schritte.map((s) => `${s.schritt}: ${s.meldung || s.ist}`).join(' | ') : JSON.stringify(d).slice(0, 160));

// --- Szene -----------------------------------------------------------------------------------------------------
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
const huelle = $('twin');
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setPixelRatio(window.devicePixelRatio);
huelle.appendChild(renderer.domElement);
const szene = new THREE.Scene();
const kamera = new THREE.PerspectiveCamera(45, 1, 0.01, 20);
kamera.position.set(0.95, -0.85, 0.65);
const steuerung = new OrbitControls(kamera, renderer.domElement);
steuerung.target.set(0.3, 0, 0.25);
szene.add(new THREE.HemisphereLight(0xffffff, 0x445566, 1.6));
const licht = new THREE.DirectionalLight(0xffffff, 1.4); licht.position.set(1, -1, 2); szene.add(licht);

const raster = new THREE.GridHelper(1.6, 16, 0x888888, 0xbbbbbb);
raster.rotation.x = Math.PI / 2; szene.add(raster);
szene.add(new THREE.AxesHelper(0.25));

function achsen(groesse, dick = false) {
  const a = new THREE.AxesHelper(groesse);
  if (dick) a.material.linewidth = 3;
  a.matrixAutoUpdate = false; szene.add(a); return a;
}
function setzeMatrix(obj, m) { // m: Zeilen (Python) → three.js Matrix4.set erwartet Zeilenreihenfolge
  obj.matrix.set(...m.flat()); obj.matrixWorldNeedsUpdate = true; obj.visible = true;
}

const lader = new STLLoader();
const material = new THREE.MeshStandardMaterial({ color: 0x9aa3ad, metalness: 0.2, roughness: 0.6 });
const greiferMaterial = new THREE.MeshStandardMaterial({ color: 0x3a3f46, metalness: 0.3, roughness: 0.5 });
const ARM = ['base_link', 'shoulder_link', 'half_arm_1_link', 'half_arm_2_link', 'forearm_link',
  'spherical_wrist_1_link', 'spherical_wrist_2_link', 'bracelet_link'];
const GREIFER = ['robotiq_base', 'left_knuckle', 'right_knuckle', 'left_finger', 'right_finger',
  'left_inner_knuckle', 'right_inner_knuckle', 'left_finger_tip', 'right_finger_tip'];
const glieder = {};
for (const name of [...ARM, ...GREIFER]) {
  const g = new THREE.Group(); g.matrixAutoUpdate = false; g.visible = false; szene.add(g); glieder[name] = g;
  const datei = ARM.includes(name)
    ? `/meshes/gen3_7dof/${name === 'bracelet_link' ? 'bracelet_with_vision_link' : name}.STL`
    : `/meshes/robotiq_2f85/${name}.stl`;
  lader.load(datei, (geo) => { geo.computeVertexNormals(); g.add(new THREE.Mesh(geo, ARM.includes(name) ? material : greiferMaterial)); });
  if (ARM.includes(name) && name !== 'base_link') { const a = new THREE.AxesHelper(0.06); g.add(a); }
}
const tcp = achsen(0.12, true);
const kameraFrame = achsen(0.05);
const tagFrame = achsen(0.06); tagFrame.visible = false;
const tagFlaeche = new THREE.Mesh(new THREE.PlaneGeometry(0.0395, 0.0395),
  new THREE.MeshBasicMaterial({ color: 0x111111, side: THREE.DoubleSide }));
tagFlaeche.matrixAutoUpdate = false; tagFlaeche.visible = false; szene.add(tagFlaeche);
const wuerfel = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1),
  new THREE.MeshStandardMaterial({ color: 0xe0a040, transparent: true, opacity: 0.6 }));
wuerfel.visible = false; szene.add(wuerfel);
const griffFrame = achsen(0.08); griffFrame.visible = false;
const simWuerfel = {}; // Simulation: wahre Lage der Würfel (nur Betreuer)
const holz = new THREE.MeshStandardMaterial({ color: 0xd2a064, roughness: 0.8 });
const raum = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(1.4, 1.4, 0.8)),
  new THREE.LineBasicMaterial({ color: 0x8899aa, transparent: true, opacity: 0.35 }));
raum.position.set(0, 0, 0.4); szene.add(raum);

function groesse() {
  const { clientWidth: b, clientHeight: h } = huelle;
  renderer.setSize(b, h, false); kamera.aspect = b / h; kamera.updateProjectionMatrix();
}
new ResizeObserver(groesse).observe(huelle); groesse();
renderer.setAnimationLoop(() => { steuerung.update(); renderer.render(szene, kamera); });

// --- Zustand ---------------------------------------------------------------------------------------------------
const f1 = (v, n = 1) => (v === null || v === undefined ? '–' : Number(v).toFixed(n));
let bewegt = false;
async function aktualisieren() {
  let z;
  try {
    const antwort = await fetch('/api/zustand', { headers: kopf() });
    z = await antwort.json();
    if (antwort.status === 401) { netzAnzeigen(null); return; }
  } catch { return; }
  if (!z.bereit) return;
  netzAnzeigen(z);
  const p = z.pose;
  [p.x * 1000, p.y * 1000, p.z * 1000, p.theta_x, p.theta_y, p.theta_z].forEach((v, i) => { $('pose').children[i].textContent = f1(v); });
  z.gelenke_deg.forEach((v, i) => { $('gelenke').children[i].textContent = f1(v); });
  $('fk').textContent = `URDF-Modell gegen gemessene Pose: ${f1(z.fk_abweichung_mm)} mm · Spannung ${f1(z.spannung_v)} V`;
  const az = $('armzustand');
  az.textContent = z.verbindungsfehler ? 'KEINE ANTWORT VOM ARM'
    : z.armzustand.replace('ARMSTATE_', '') + (z.fehler ? ' · FEHLER' : '') + (z.neu_verbunden ? ` · neu verbunden ${z.neu_verbunden}` : '');
  az.title = z.verbindungsfehler || '';
  az.className = 'chip' + (z.verbindungsfehler || z.fehler || z.armzustand === 'ARMSTATE_IN_FAULT' ? ' fehler' : '');
  bewegt = z.bewegt; $('bewegt').textContent = z.bewegt ? 'bewegt sich' : 'steht'; $('bewegt').className = 'chip' + (z.bewegt ? ' warn' : '');
  $('weite_ist').textContent = f1(z.greifer_weite_mm);
  const knapp = Object.entries(z.anschlag_reserve_deg || {}).filter(([, r]) => r < 10);
  $('anschlag').textContent = knapp.length ? `Anschlag nah: ${knapp.map(([n, r]) => `Gelenk ${n} ${f1(r)}°`).join(', ')}` : 'Gelenke 2/4/6 frei';
  $('home').textContent = (z.home ? `Home vom ${z.home.gespeichert}: ${z.home.gelenke_deg.map((v) => f1(v)).join(', ')}°` : 'keine Home-Pose gespeichert') +
    (z.park ? `\nPark vom ${z.park.gespeichert}: ${z.park.gelenke_deg.map((v) => f1(v)).join(', ')}°` : '');
  for (const [name, m] of Object.entries(z.glieder)) if (glieder[name]) setzeMatrix(glieder[name], m);
  setzeMatrix(tcp, z.frames.werkzeug_gemessen); setzeMatrix(kameraFrame, z.frames.kamera_farbe);
  for (const name of ['farbe', 'tiefe']) {
    const knopf = document.querySelector(`[data-stream=${name}]`); const an = z.streams[name];
    knopf.textContent = an ? 'aus' : 'an';
    const zugang = team.schluessel ? `schluessel=${encodeURIComponent(team.schluessel)}&` : '';
    const bild = $(`bild_${name}`); if (an && !bild.src.includes('.mjpg')) bild.src = `/video/${name}.mjpg?${zugang}t=${Date.now()}`;
    if (!an && bild.src) bild.removeAttribute('src');
  }
  if (z.tag && z.tag.basis_matrix) {
    setzeMatrix(tagFrame, z.tag.basis_matrix); setzeMatrix(tagFlaeche, z.tag.basis_matrix);
    const b = z.tag.basis;
    $('tag').textContent = `ID ${z.tag.id}: x ${f1(b.x * 1000)} y ${f1(b.y * 1000)} z ${f1(b.z * 1000)} mm, Abstand Kamera ${f1(z.tag.abstand_m * 1000)} mm, Reprojektion ${f1(z.tag.reprojektion_px, 2)} px`;
  }
  $('ablauf').textContent = ablaufText(z.ablauf) + (z.haelt_wuerfel ? '\nWürfel im Greifer' : '');
  $('kalibrierung').textContent = kalibrierText(z.farbkamera, z.kalibrierung);
  if (z.wuerfel) {
    const w = z.wuerfel;
    wuerfel.scale.set(w.kanten_mm[0] / 1000, w.kanten_mm[1] / 1000, w.hoehe_mm / 1000);
    wuerfel.position.set(...w.mitte_m); wuerfel.rotation.set(0, 0, w.gier_deg * Math.PI / 180); wuerfel.visible = true;
    $('wuerfel').textContent = `Kanten ${w.kanten_mm.join(' × ')} mm, Höhe ${w.hoehe_mm} mm, Mitte (${w.mitte_m.map((v) => f1(v * 1000)).join(', ')}) mm, Gier ${w.gier_deg}°, Tisch z ${w.tisch_z_mm} mm\nGriff: Fingerspitzen bei z ${w.spitze_z_mm} mm, Greifer offen ${w.greifer_offen_mm} mm` +
      (w.pruefung_griff.length ? `\nGriff verboten: ${w.pruefung_griff.join('; ')}` : '');
  }
}
setInterval(aktualisieren, 150);

// --- Netzzugang und Simulation ------------------------------------------------------------------------------------
function netzAnzeigen(z) {
  const betreuer = z && z.rolle === 'betreuer';
  $('netz_betreuer').hidden = !betreuer; $('netz_team').hidden = betreuer;
  if (!z) { $('netz_team_info').textContent = 'Team-Schlüssel eintragen (steht in euren Geräteangaben).'; return; }
  const n = z.netz; const f = n.fahrer;
  const fahrer = f ? `${f.team} (seit ${f.seit_s} s${f.laeuft ? ', Befehl läuft' : `, ruhig seit ${f.ruhig_s} s`})` : 'niemand';
  if (betreuer) {
    $('netz_freigabe').textContent = n.freigabe ? 'Freigabe zurücknehmen (hält an)' : 'Fahren für Teams freigeben';
    $('netz_freigabe').className = n.freigabe ? 'warnknopf' : '';
    $('netz_info').textContent = `Fahren übers Netz: ${n.freigabe ? 'FREIGEGEBEN' : 'gesperrt'} · Fahrrecht: ${fahrer}
` +
      `Team-Schlüssel: ${n.schluessel}` + (n.adressen.length ? ` · Adressen: ${n.adressen.join(', ')}` : ' · nur lokal (Server mit --netz starten)') +
      (n.ereignisse.length ? `
${n.ereignisse.slice(-4).map((e) => `${e.zeit} ${e.text}`).join('\n')}` : '');
  } else {
    $('netz_team_info').textContent = `Fahren ${n.freigabe ? 'freigegeben' : 'noch nicht freigegeben'} · Fahrrecht: ${fahrer}` +
      (n.eigenes_fahrrecht ? ' (ihr)' : '');
  }
  const sim = z.simulation;
  $('sim_teil').hidden = !sim;
  if (sim) $('sim_meldung').textContent = sim.meldung ? `Simulation: ${sim.meldung}` : 'Simulation';
  const gesehen = new Set();
  for (const w of (sim && sim.wuerfel) || []) {
    gesehen.add(w.id);
    if (!simWuerfel[w.id]) { simWuerfel[w.id] = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), holz); szene.add(simWuerfel[w.id]); }
    const m = simWuerfel[w.id]; const k = w.kante_mm / 1000;
    m.scale.set(k, k, k); m.position.set(...w.mitte_mm.map((v) => v / 1000)); m.rotation.set(0, 0, w.gier_deg * Math.PI / 180);
  }
  for (const [id, m] of Object.entries(simWuerfel)) m.visible = gesehen.has(Number(id));
}
$('netz_freigabe').addEventListener('click', async () => {
  const an = !$('netz_freigabe').textContent.startsWith('Freigabe zurück');
  if (!an || confirm('Fahren für Teams freigeben? Das Team mit Fahrrecht kann den Arm dann bewegen und greifen.')) post('/api/netz/freigabe', { an }).catch(() => {});
});
$('netz_entziehen').addEventListener('click', () => post('/api/netz/entziehen').catch(() => {}));
$('netz_schluessel').value = team.schluessel || ''; $('netz_teamname').value = team.name || '';
$('netz_anmelden').addEventListener('click', async () => {
  team.schluessel = $('netz_schluessel').value.trim() || null; team.name = $('netz_teamname').value.trim();
  merken(localStorage, 'kinova_schluessel', team.schluessel); merken(localStorage, 'kinova_team', team.name);
  team.fahrrecht = null;
  const d = await post('/api/fahrrecht/anmelden', { team: team.name }).catch(() => null);
  team.fahrrecht = d ? d.fahrrecht : null; merken(sessionStorage, 'kinova_fahrrecht', team.fahrrecht);
});
$('netz_abmelden').addEventListener('click', async () => {
  await post('/api/fahrrecht/abmelden').catch(() => {}); team.fahrrecht = null; merken(sessionStorage, 'kinova_fahrrecht', null);
});
$('sim_szene').addEventListener('click', () => post('/api/sim/szene', {}).catch(() => {}));
$('sim_zufall').addEventListener('click', () => post('/api/sim/szene', { zufall: Math.floor(Math.random() * 1000) }).catch(() => {}));

function kalibrierText(fk, k) {
  const aktiv = fk === 'Werkswerte' ? 'Werkswerte (Kinova-Vorgabe, URDF-Nennmaße)'
    : `Kalibrierung vom ${fk.kalibriert} (${f1(fk.guete.rms_px, 2)} px, Streuung bis ${f1(fk.guete.streuung_max_mm, 1)} mm)`;
  if (!k) return `Aktiv: ${aktiv}`;
  const g = k.guete; const a = k.aenderung; const i = k.intrinsik;
  return `Aktiv: ${aktiv}\nLetzte Auswertung: ${g.ansichten} Ansichten, Bildfehler ${f1(g.rms_px, 2)} px, Streuung bis ` +
    `${f1(g.streuung_max_mm, 1)} mm` +
    (g.gegenprobe_rms_px != null ? `, Gegenprobe ${f1(g.gegenprobe_rms_px, 2)} px` : '') +
    ` → ${g.brauchbar ? 'brauchbar' : 'NICHT brauchbar'}\n` +
    (k.abdeckung ? `Bildfelder ohne Ecken: ${k.abdeckung.leere_felder} von 40, unter 10 Ecken: ${k.abdeckung.felder_unter_10}\n` : '') +
    (k.ausgelassen && k.ausgelassen.length ? `Ausreißer ausgelassen: ${k.ausgelassen.map((x) => `${x.datei} (${f1(x.fehler_px, 1)} px)`).join(', ')}\n` : '') +
    `fx ${f1(i.fx)} fy ${f1(i.fy)} cx ${f1(i.cx)} cy ${f1(i.cy)} · Kamera gegen vorher ${f1(a.betrag_mm)} mm, ${f1(a.drehung_deg, 2)}°\n` +
    `Brettoberfläche z ${f1(k.brett.oberflaeche_z_mm)} mm, Neigung ${f1(k.brett.neigung_deg, 2)}°` +
    (k.tiefe_gegen_farbe ? ` · Tiefenbild − Farbe ${f1(k.tiefe_gegen_farbe.abstand_mm)} mm` : '');
}

// --- Bedienung -------------------------------------------------------------------------------------------------
const bezug = () => document.querySelector('input[name=bezug]:checked').value;
document.querySelectorAll('[data-achse]').forEach((k) => k.addEventListener('click', () => {
  const achse = k.dataset.achse; const v = Number(k.dataset.v);
  const wert = achse.startsWith('dr') ? v * Number($('winkel').value) : v * Number($('weg').value) / 1000;
  post('/api/schritt', { bezug: bezug(), [achse]: wert }).catch(() => {});
}));
document.querySelectorAll('[data-gelenk]').forEach((k) => k.addEventListener('click', () =>
  post('/api/gelenk', { gelenk: Number($('gelenk_nr').value), d_deg: Number(k.dataset.gelenk) * Number($('winkel').value) }).catch(() => {})));
document.querySelectorAll('[data-post]').forEach((k) => k.addEventListener('click', () => post(k.dataset.post).catch(() => {})));
document.querySelectorAll('[data-stream]').forEach((k) => k.addEventListener('click', () => {
  const an = k.textContent === 'an'; post(`/api/stream/${k.dataset.stream}/${an ? 'an' : 'aus'}`).catch(() => {});
}));
document.querySelectorAll('[data-tag]').forEach((k) => k.addEventListener('click', () =>
  post('/api/apriltag/anfahren', { abstand_spitze_m: Number(k.dataset.tag),
    ausrichtung: $('tag_normale').checked ? 'tag' : 'senkrecht' }).catch(() => {})));
$('stopp').addEventListener('click', () => post('/api/stopp').catch(() => {}));
$('home_speichern').addEventListener('click', () => {
  if (confirm('Aktuelle Pose als Home-Pose speichern?')) post('/api/home/speichern').catch(() => {});
});
$('park_speichern').addEventListener('click', () => {
  if (confirm('Aktuelle Pose als Parkposition speichern?')) post('/api/park/speichern').catch(() => {});
});
$('park_fahren').addEventListener('click', () => {
  if (confirm('Parkposition anfahren (Gelenkfahrt, vorher simuliert)? Arbeitsbereich frei?')) post('/api/park/fahren').catch(() => {});
});
const tempo = () => Number($('greifer_tempo').value) / 100;
$('greifer_tempo').addEventListener('input', () => { $('tempo_wert').textContent = $('greifer_tempo').value; });
$('greifer_weite').addEventListener('input', () => { $('weite_wert').textContent = $('greifer_weite').value; });
$('greifer_auf').addEventListener('click', () => post('/api/greifer', { position: 0, geschwindigkeit: tempo() }).catch(() => {}));
$('greifer_zu').addEventListener('click', () => post('/api/greifer', { position: 1, geschwindigkeit: tempo() }).catch(() => {}));
$('weite_anfahren').addEventListener('click', () => post('/api/greifer', { weite_mm: Number($('greifer_weite').value), geschwindigkeit: tempo() }).catch(() => {}));
$('dabei').addEventListener('change', () => {
  document.querySelectorAll('.bestaetigen').forEach((k) => { k.disabled = !$('dabei').checked; });
});
document.querySelectorAll('.bestaetigen[data-skill]').forEach((k) => k.addEventListener('click', () => {
  if (confirm(k.dataset.frage)) post(k.dataset.skill, { bestaetigt: true }).catch(() => {});
}));
$('schnell_stapeln').addEventListener('click', () => {
  const stufe = $('schnell_stufe').value;
  const bahn = $('schnell_bahn').checked;
  const direkt = $('schnell_direkt').checked;
  if (confirm(`Schnell stapeln, Stufe ${stufe}${bahn ? ', Gelenkbahn' : ''}${direkt ? ', ohne Einzelscan' : ''}? `
      + 'Der Arm fährt deutlich schneller als sonst. Arbeitsbereich frei, Hand am Not-Halt?'))
    post('/api/wuerfel/stapeln_schnell', { bestaetigt: true, stufe, bahn, direkt }).catch(() => {});
});
$('kal_start').addEventListener('click', () => {
  if (confirm('Farbkamera kalibrieren: Home, dann 33 Gelenkfahrten über dem Brett (jede vorher simuliert, ≈ 3 min), zurück nach Home. Arbeitsbereich frei?'))
    post('/api/kalibrierung/aufnehmen', { quadrat_mm: Number($('quadrat_mm').value) }).catch(() => {});
});
$('kal_rechnen').addEventListener('click', () => post('/api/kalibrierung/rechnen', { quadrat_mm: Number($('quadrat_mm').value) }).catch(() => {}));
$('kal_uebernehmen').addEventListener('click', () => {
  if (confirm('Kalibrierung übernehmen? AprilTag-Erkennung und Kamerabild im Twin nutzen danach die neuen Werte.')) post('/api/kalibrierung/uebernehmen').catch(() => {});
});
$('kal_werk').addEventListener('click', () => {
  if (confirm('Zurück zu Kinovas Werkswerten für die Farbkamera?')) post('/api/kalibrierung/werkswerte').catch(() => {});
});
const ablaufText = (a) => {
  if (!a || (!a.laeuft && !a.verlauf?.length)) return 'kein Ablauf';
  const letzte = a.verlauf.slice(-4).map((s) => `${s.zeit} ${s.schritt}${s.ergebnis ? ` → ${s.ergebnis}` : ''}`).join('\n');
  return `${a.laeuft ? '▶ läuft' : (a.fehler ? '✗ abgebrochen' : '✓ fertig')}: ${a.name || ''}${a.fehler ? ` (${a.fehler})` : ''}\n${letzte}`;
};
