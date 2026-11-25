/**
 * 二级圆柱齿轮减速器 · 运动学仿真
 * ------------------------------------------------
 * 传动链：电机 → 输入轴 → 小齿轮1 → 大齿轮2（中间轴）
 *         → 同轴小齿轮3 → 大齿轮4 → 输出轴
 *
 * 说明：齿轮采用参数化生成（同一模数、齿数成对），
 * 这是保证"严格按传动比啮合、齿面不穿模"的关键——
 * 外部下载的通用 glb 齿轮模型齿数/模数任意，无法满足该精度要求。
 * 仍内置 GLTFLoader：若 models/ 下存在 motor.glb / housing.glb，
 * 会自动加载外部模型替换对应的程序化部件（见文件末尾 loadExternalModels）。
 */

import * as THREE from 'three';
import { OrbitControls } from './vendor/jsm/controls/OrbitControls.js';
import { GLTFLoader } from './vendor/jsm/loaders/GLTFLoader.js';

/* ================= 传动参数 ================= */
const MODULE = 2;            // 模数（所有齿轮相同，啮合前提）
const Z1 = 18, Z2 = 54;      // 第一级：小齿轮 / 大齿轮齿数，传动比 3
const Z3 = 18, Z4 = 54;      // 第二级：小齿轮 / 大齿轮齿数，传动比 3
const RATIO = (Z2 / Z1) * (Z4 / Z3);   // 总传动比 9
const GEAR_T = 16;           // 齿宽
const Z_STAGE1 = 14, Z_STAGE2 = -14;   // 两级齿轮在轴向上的位置

// 三根轴的位置（轴线平行于 Z 轴，齿轮端面朝向观察者）
const SHAFT_IN  = new THREE.Vector3(-(MODULE * (Z1 + Z2) / 2), 0, 0); // 输入轴 x=-72
const SHAFT_MID = new THREE.Vector3(0, 0, 0);                          // 中间轴 x=0
const SHAFT_OUT = new THREE.Vector3(MODULE * (Z3 + Z4) / 2, 0, 0);     // 输出轴 x=+72

/* ================= 渲染器 / 场景 ================= */
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(innerWidth, innerHeight);
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;   // 柔和阴影
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.1;
document.body.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x14171b);
scene.fog = new THREE.Fog(0x14171b, 380, 900);      // 淡雾提升空间感

const camera = new THREE.PerspectiveCamera(42, innerWidth / innerHeight, 0.1, 2000);
camera.position.set(90, 80, 300);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, -5, 0);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.maxDistance = 700;
controls.update();

/* ================= 灯光 ================= */
scene.add(new THREE.HemisphereLight(0x9db8d9, 0x2a241c, 0.55)); // 环境补光

const sun = new THREE.DirectionalLight(0xffffff, 2.2);
sun.position.set(150, 220, 170);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
sun.shadow.camera.left = sun.shadow.camera.bottom = -200;
sun.shadow.camera.right = sun.shadow.camera.top = 200;
sun.shadow.camera.near = 50;
sun.shadow.camera.far = 700;
sun.shadow.bias = -0.0005;
scene.add(sun);

/* ================= 材质 ================= */
const MAT = {
  gear:    new THREE.MeshStandardMaterial({ color: 0x9aa2ab, metalness: 0.85, roughness: 0.35 }),
  pinion:  new THREE.MeshStandardMaterial({ color: 0x7d8894, metalness: 0.85, roughness: 0.4 }),
  shaft:   new THREE.MeshStandardMaterial({ color: 0xb8bec6, metalness: 0.9,  roughness: 0.25 }),
  housing: new THREE.MeshStandardMaterial({ color: 0x3a4048, metalness: 0.6,  roughness: 0.6 }),
  base:    new THREE.MeshStandardMaterial({ color: 0x2c3138, metalness: 0.5,  roughness: 0.7 }),
  motor:   new THREE.MeshStandardMaterial({ color: 0x4a6b8a, metalness: 0.7,  roughness: 0.45 }),
  dark:    new THREE.MeshStandardMaterial({ color: 0x23272d, metalness: 0.6,  roughness: 0.6 }),
  marker:  new THREE.MeshStandardMaterial({ color: 0xff8c3b, emissive: 0x903c00, roughness: 0.5 }),
};

/* ================= 齿轮参数化建模 =================
 * 梯形近似齿形：同一模数下，配对齿轮的周节相同，
 * 配合初始相位对齐，可保证全程啮合同步、不穿模。 */
function createGear(teeth, thickness, mat) {
  const rPitch = MODULE * teeth / 2;        // 分度圆半径
  const rTip = rPitch + MODULE;             // 齿顶圆
  const rRoot = rPitch - 1.25 * MODULE;     // 齿根圆
  const p = Math.PI * 2 / teeth;            // 角齿距
  const aRoot = p * 0.26, aTip = p * 0.16;  // 齿根/齿顶半角（留侧隙）

  const shape = new THREE.Shape();
  for (let i = 0; i < teeth; i++) {
    const a = i * p; // 每个齿的中心角（齿 0 正对 +X，便于相位对齐）
    const pts = [
      [rRoot, a - aRoot], [rTip, a - aTip],
      [rTip, a + aTip], [rRoot, a + aRoot],
    ];
    for (const [r, ang] of pts) {
      const x = r * Math.cos(ang), y = r * Math.sin(ang);
      if (i === 0 && ang === pts[0][1]) shape.moveTo(x, y);
      else shape.lineTo(x, y);
    }
    // 齿根圆弧过渡到下一个齿
    const mid = a + p / 2;
    shape.lineTo(rRoot * Math.cos(mid), rRoot * Math.sin(mid));
  }
  shape.closePath();

  // 中心轴孔
  const bore = new THREE.Path();
  bore.absarc(0, 0, 7.5, 0, Math.PI * 2, true);
  shape.holes.push(bore);

  // 大齿轮腹板减重孔
  if (teeth >= 40) {
    for (let k = 0; k < 5; k++) {
      const a = k * Math.PI * 2 / 5;
      const hole = new THREE.Path();
      hole.absarc(32 * Math.cos(a), 32 * Math.sin(a), 8, 0, Math.PI * 2, true);
      shape.holes.push(hole);
    }
  }

  const geo = new THREE.ExtrudeGeometry(shape, {
    depth: thickness, bevelEnabled: true,
    bevelThickness: 0.5, bevelSize: 0.5, bevelSegments: 1,
  });
  geo.translate(0, 0, -thickness / 2);

  const gear = new THREE.Group();
  const mesh = new THREE.Mesh(geo, mat);
  mesh.castShadow = mesh.receiveShadow = true;
  gear.add(mesh);

  // 轮毂
  const hub = new THREE.Mesh(
    new THREE.CylinderGeometry(teeth >= 40 ? 16 : 11, teeth >= 40 ? 16 : 11, thickness + 8, 24), mat);
  hub.rotation.x = Math.PI / 2;
  hub.castShadow = true;
  gear.add(hub);

  // 橙色标记点：便于观察转动与啮合位置
  const dot = new THREE.Mesh(new THREE.SphereGeometry(2.2, 12, 12), MAT.marker);
  dot.position.set(rTip - 5, 0, thickness / 2 + 2);
  gear.add(dot);

  return gear;
}

/* ================= 轴 / 箱体 / 电机 ================= */
function makeShaft(r, z0, z1) {
  const len = z1 - z0;
  const s = new THREE.Mesh(new THREE.CylinderGeometry(r, r, len, 24), MAT.shaft);
  s.rotation.x = Math.PI / 2;
  s.position.z = (z0 + z1) / 2;
  s.castShadow = true;
  return s;
}
function box(w, h, d, x, y, z, mat) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat);
  m.position.set(x, y, z);
  m.castShadow = m.receiveShadow = true;
  return m;
}

// --- 底座与地面 ---
scene.add(box(330, 10, 120, 0, -80, 0, MAT.base));
const ground = new THREE.Mesh(new THREE.PlaneGeometry(2000, 2000),
  new THREE.MeshStandardMaterial({ color: 0x1a1d21, roughness: 0.95 }));
ground.rotation.x = -Math.PI / 2;
ground.position.y = -85;
ground.receiveShadow = true;
scene.add(ground);

// --- 箱体（开放前/顶面，便于观察内部啮合） ---
const housing = new THREE.Group();
housing.add(box(288, 150, 4, 0, 0, -34, MAT.housing));   // 后壁
housing.add(box(4, 150, 72, -142, 0, 0, MAT.housing));   // 左壁
housing.add(box(4, 150, 72, 142, 0, 0, MAT.housing));    // 右壁
housing.add(box(288, 4, 72, 0, -73, 0, MAT.housing));    // 底壁
scene.add(housing);

// 后壁轴承座
for (const sx of [SHAFT_IN, SHAFT_MID, SHAFT_OUT]) {
  const brg = new THREE.Mesh(new THREE.TorusGeometry(11, 3.5, 12, 32), MAT.dark);
  brg.position.set(sx.x, 0, -31);
  scene.add(brg);
}

// --- 三根轴 ---
const inShaft = makeShaft(6, -58, 30); inShaft.position.x = SHAFT_IN.x; scene.add(inShaft);
const midShaft = makeShaft(8, -30, 30); midShaft.position.x = SHAFT_MID.x; scene.add(midShaft);
const outShaft = makeShaft(7, -30, 58); outShaft.position.x = SHAFT_OUT.x; scene.add(outShaft);

// 输出轴前端支撑（立柱 + 轴承环）
scene.add(box(18, 75, 6, SHAFT_OUT.x, -37.5, 50, MAT.housing));
const outBrg = new THREE.Mesh(new THREE.TorusGeometry(10, 3, 12, 32), MAT.dark);
outBrg.position.set(SHAFT_OUT.x, 0, 50);
scene.add(outBrg);

// --- 电机（程序化默认模型，可被 models/motor.glb 替换） ---
function createMotor() {
  const g = new THREE.Group();
  const body = new THREE.Mesh(new THREE.CylinderGeometry(20, 20, 36, 32), MAT.motor);
  body.rotation.x = Math.PI / 2;
  body.castShadow = true;
  g.add(body);
  for (let k = 0; k < 10; k++) {          // 散热筋
    const fin = box(1.6, 42, 30, 0, 0, 0, MAT.motor);
    fin.rotation.z = k * Math.PI / 10;
    g.add(fin);
  }
  const cap = new THREE.Mesh(new THREE.CylinderGeometry(13, 16, 8, 24), MAT.dark);
  cap.rotation.x = Math.PI / 2; cap.position.z = 21; cap.castShadow = true;
  g.add(cap);
  g.add(box(16, 12, 18, 0, 24, 0, MAT.dark));   // 接线盒
  g.add(box(30, 8, 26, 0, -24, 0, MAT.dark));   // 底脚
  return g;
}
const motor = createMotor();
motor.position.set(SHAFT_IN.x, 0, -56);
scene.add(motor);

/* ================= 装配四级齿轮 ================= */
const gear1 = createGear(Z1, GEAR_T, MAT.pinion);            // 一级小齿轮（输入）
gear1.position.set(SHAFT_IN.x, 0, Z_STAGE1);
const gear2 = createGear(Z2, GEAR_T, MAT.gear);              // 一级大齿轮（中间轴）
gear2.position.set(SHAFT_MID.x, 0, Z_STAGE1);
const gear3 = createGear(Z3, GEAR_T, MAT.pinion);            // 二级小齿轮（中间轴）
gear3.position.set(SHAFT_MID.x, 0, Z_STAGE2);
const gear4 = createGear(Z4, GEAR_T, MAT.gear);              // 二级大齿轮（输出）
gear4.position.set(SHAFT_OUT.x, 0, Z_STAGE2);
scene.add(gear1, gear2, gear3, gear4);

/* ================= 啮合相位标定 =================
 * 齿形生成时令"齿 0 正对 +X"。配对齿轮位于其 +X 方向时，
 * 对方需在 -X 方向（自身角度 π）对准一个齿槽：
 * 齿槽中心角 = (2k+1)·π/z + φ，取 k=0 得 φ = π - π/z。 */
const PHASE2 = Math.PI - Math.PI / Z2;   // 齿轮2 相对齿轮1 的初相
const PHASE4 = Math.PI - Math.PI / Z4;   // 齿轮4 相对齿轮3 的初相
// 齿轮3 与齿轮2 同轴固连，附加固定偏置使 θ1=0 时齿轮3 的齿正对 +X
const OFFSET3 = -PHASE2;

/** 由输入轴转角 θ1 推导全部齿轮转角（纯运动学，严格按传动比） */
function applyKinematics(theta1) {
  gear1.rotation.z = theta1;
  gear2.rotation.z = -theta1 * (Z1 / Z2) + PHASE2;   // 外啮合反向，ω2 = -ω1·z1/z2
  gear3.rotation.z = gear2.rotation.z + OFFSET3;     // 同轴同步
  gear4.rotation.z = -gear3.rotation.z * (Z3 / Z4) + PHASE4;
}

/* ================= 动力流粒子 =================
 * 沿"电机→输入轴→啮合点1→中间轴→啮合点2→输出轴"路径流动 */
const flowPath = new THREE.CatmullRomCurve3([
  new THREE.Vector3(SHAFT_IN.x, 0, -70),
  new THREE.Vector3(SHAFT_IN.x, 0, Z_STAGE1),
  new THREE.Vector3(SHAFT_IN.x + MODULE * Z1 / 2, 0, Z_STAGE1), // 啮合点1
  new THREE.Vector3(SHAFT_MID.x, 0, Z_STAGE1),
  new THREE.Vector3(SHAFT_MID.x, 0, Z_STAGE2),
  new THREE.Vector3(SHAFT_MID.x + MODULE * Z3 / 2, 0, Z_STAGE2), // 啮合点2
  new THREE.Vector3(SHAFT_OUT.x, 0, Z_STAGE2),
  new THREE.Vector3(SHAFT_OUT.x, 0, 56),
]);
const PARTICLES = 90;
const pGeo = new THREE.BufferGeometry();
pGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(PARTICLES * 3), 3));
const pTex = (() => {   // 圆形光点贴图
  const c = document.createElement('canvas'); c.width = c.height = 64;
  const ctx = c.getContext('2d');
  const g = ctx.createRadialGradient(32, 32, 2, 32, 32, 30);
  g.addColorStop(0, 'rgba(255,170,80,1)'); g.addColorStop(1, 'rgba(255,140,40,0)');
  ctx.fillStyle = g; ctx.fillRect(0, 0, 64, 64);
  return new THREE.CanvasTexture(c);
})();
const particles = new THREE.Points(pGeo, new THREE.PointsMaterial({
  map: pTex, size: 5, transparent: true, depthWrite: false,
  blending: THREE.AdditiveBlending, sizeAttenuation: true,
}));
particles.visible = false;
scene.add(particles);

let flowS = 0;   // 粒子路径参数里程
function updateParticles() {
  const pos = pGeo.attributes.position;
  for (let i = 0; i < PARTICLES; i++) {
    const u = ((flowS + i / PARTICLES) % 1 + 1) % 1;
    const p = flowPath.getPointAt(u);
    pos.setXYZ(i, p.x, p.y, p.z);
  }
  pos.needsUpdate = true;
}

/* ================= 仿真状态 / UI ================= */
let running = true;        // 默认自动运转
let theta1 = 0;            // 输入轴累计转角（所有齿轮转角的唯一驱动量）
let inputRpm = 60;

const $ = id => document.getElementById(id);
const btnToggle = $('btnToggle'), btnFlow = $('btnFlow');

btnToggle.onclick = () => {
  running = !running;
  btnToggle.textContent = running ? '暂停' : '继续';
  btnToggle.classList.toggle('active', !running);
};
$('btnReset').onclick = () => {           // 重置：转角与粒子归零
  theta1 = 0; flowS = 0;
  applyKinematics(0); updateParticles();
};
btnFlow.onclick = () => {
  particles.visible = !particles.visible;
  btnFlow.classList.toggle('active', particles.visible);
};
$('rpm').oninput = e => {
  inputRpm = +e.target.value;
  $('rpmVal').textContent = inputRpm;
  $('outRpm').textContent = (inputRpm / RATIO).toFixed(1);
};

/* ================= 可选：加载外部 glb 模型 =================
 * 将 motor.glb / housing.glb 放入 models/ 即自动替换程序化部件 */
function loadExternalModels() {
  const loader = new GLTFLoader();
  const tryLoad = url => new Promise(res =>
    loader.load(url, g => res(g.scene), undefined, () => res(null)));

  tryLoad('./models/motor.glb').then(m => {
    if (!m) return;
    m.traverse(o => { if (o.isMesh) o.castShadow = true; });
    scene.remove(motor);
    m.position.copy(motor.position);
    scene.add(m);
  });
  tryLoad('./models/housing.glb').then(m => {
    if (!m) return;
    m.traverse(o => { if (o.isMesh) { o.castShadow = o.receiveShadow = true; } });
    scene.remove(housing);
    scene.add(m);
  });
}
loadExternalModels();

/* ================= 动画循环 ================= */
const clock = new THREE.Clock();
applyKinematics(0);

function animate() {
  requestAnimationFrame(animate);
  const dt = Math.min(clock.getDelta(), 0.05);

  if (running) {
    const omega = inputRpm * Math.PI / 30;   // rpm → rad/s
    theta1 += omega * dt;
    applyKinematics(theta1);
    flowS += omega * dt * 0.02;              // 粒子流速跟随输入转速
    if (particles.visible) updateParticles();
  }

  controls.update();
  renderer.render(scene, camera);
}
animate();

addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});
