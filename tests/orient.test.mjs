// orient.test.mjs — 独立交叉验证 js/orient.js 的角度→姿态映射
// 方法: 用「四元数」以不同形式复现同一约定, 与 orient.js(Rodrigues 迭代)比对;
//       再对若干解析姿态点做几何断言, 检查正交性/右手性/映射到 CSS 观察空间。
import assert from 'node:assert/strict';
import orient from '../js/orient.js';

const { deviceFrameFromAngles } = orient;
const DEG = Math.PI / 180;
const V = (x,y,z)=>[x,y,z];
const dot = (a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const add=(a,b)=>[a[0]+b[0],a[1]+b[1],a[2]+b[2]];
const scale=(s,a)=>[a[0]*s,a[1]*s,a[2]*s];
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
function norm(a){const l=Math.hypot(...a)||1;return scale(1/l,a);}
function rotQuatAxis(axis, theta){ // axis: unit vector(世界系)
  const h=theta/2, s=Math.sin(h);
  return { w:Math.cos(h), x:axis[0]*s, y:axis[1]*s, z:axis[2]*s };
}
function qmul(a,b){
  return { w:a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
           x:a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
           y:a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
           z:a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w };
}
function qrot(q, v){ // 用四元数旋转向量 v(世界系)
  const t = {w:0, x:v[0], y:v[1], z:v[2]};
  const m = qmul(qmul(q, t), {w:q.w, x:-q.x, y:-q.y, z:-q.z});
  return [m.x, m.y, m.z];
}
// 用四元数实现同一约定:
//  α 绕世界 Z 转(前乘), 随后 β 绕当前机体 x 转(后乘 qX), 再 γ 绕当前机体 y 转(后乘 qY)
function quatDeviceAxes(aDeg,bDeg,gDeg){
  let q = {w:1,x:0,y:0,z:0};
  q = qmul(rotQuatAxis(V(0,0,1), aDeg*DEG), q);           // α about world Z
  q = qmul(q, rotQuatAxis(V(1,0,0), bDeg*DEG));           // β about body x
  q = qmul(q, rotQuatAxis(V(0,1,0), gDeg*DEG));           // γ about body y
  return {
    x: qrot(q, V(1,0,0)),
    y: qrot(q, V(0,1,0)),
    z: qrot(q, V(0,0,1))
  };
}

const approx = (a,b,eps=1e-9)=>{
  assert.ok(Math.abs(a[0]-b[0])<eps && Math.abs(a[1]-b[1])<eps && Math.abs(a[2]-b[2])<eps,
    'mismatch: got ' + JSON.stringify(a) + ' want ' + JSON.stringify(b));
};

// 1) 随机角度: orient.js vs 四元数复现
let rng = 12345;
const rand = () => { rng = (rng*1103515245+12345) & 0x7fffffff; return rng/0x7fffffff; };
for (let i=0;i<2000;i++){
  const a=rand()*360, b=rand()*360-180, g=rand()*180-90;
  const o = deviceFrameFromAngles(a,b,g).earth;
  const q = quatDeviceAxes(a,b,g);
  approx(o.x,q.x); approx(o.y,q.y); approx(o.z,q.z);
}

// 2) 解析姿态点
// (0,0,0): 平放, 屏幕朝上, 顶朝北 → 设备系 == 地球系 ENU
let o = deviceFrameFromAngles(0,0,0).earth;
approx(o.x,V(1,0,0)); approx(o.y,V(0,1,0)); approx(o.z,V(0,0,1));
// (0,90,0): 竖立, 顶朝上, 屏幕法线指向南(朝向站在南面的观察者)
o = deviceFrameFromAngles(0,90,0).earth;
approx(o.x,V(1,0,0)); approx(o.y,V(0,0,1)); approx(o.z,V(0,-1,0));
// (90,0,0): 仍平放, 逆时针(俯视)转 90° → 顶朝西
o = deviceFrameFromAngles(90,0,0).earth;
approx(o.x,V(0,1,0)); approx(o.y,V(-1,0,0)); approx(o.z,V(0,0,1));
// (0,180,0): 翻转平放, 屏幕朝下, 顶朝南
o = deviceFrameFromAngles(0,180,0).earth;
approx(o.x,V(1,0,0)); approx(o.y,V(0,-1,0)); approx(o.z,V(0,0,-1));
// (0,90,-90): 竖立后向左躺 → 屏幕法线朝西
o = deviceFrameFromAngles(0,90,-90).earth;
approx(o.z,V(-1,0,0));

// 3) 正交性 & 右手性 (x×y=z), 及地球系各姿态
for (let i=0;i<200;i++){
  const a=rand()*360, b=rand()*180, g=rand()*180-90;
  const { x,y,z } = deviceFrameFromAngles(a,b,g).earth;
  assert.ok(Math.abs(dot(x,y))<1e-9 && Math.abs(dot(y,z))<1e-9 && Math.abs(dot(z,x))<1e-9, 'orthogonal');
  const c = cross(x,y); approx(c,z,1e-9);
  const lx=Math.hypot(...x); assert.ok(Math.abs(lx-1)<1e-9, 'unit x');
}

// 4) CSS3D 变换空间映射: css=(east, -up, -north), y 向下 z 朝观察者(CSS 原生坐标)
const css0 = deviceFrameFromAngles(0,90,0).css; // 竖立面对观察者
approx(css0.x,V(1,0,0));  // 右 → +x
approx(css0.y,V(0,-1,0)); // 顶(上) → CSS -y
approx(css0.z,V(0,0,1));  // 屏幕法线 → 朝观察者(+z)
const cssF = deviceFrameFromAngles(0,0,0).css;  // 平放屏朝上
approx(cssF.z,V(0,-1,0)); // 屏幕法线(上) → CSS -y
approx(cssF.y,V(0,0,-1)); // 顶(北) 指向屏幕深处

// 5) 行主序矩阵与 CSS matrix3d 字符串
const m = deviceFrameFromAngles(23,64,-31);
assert.equal(m.rowMajor9.length, 9);
const cssM = orient.cssMatrix3dFromCssAxes(m.css);
assert.ok(cssM.startsWith('matrix3d('));
const nums = cssM.slice(9,-1).split(',').map(Number);
assert.equal(nums.length, 16);
// 前 3 列分别应为 css.x / css.y / css.z(列主序)
approx([nums[0],nums[1],nums[2]], m.css.x);
approx([nums[4],nums[5],nums[6]], m.css.y);
approx([nums[8],nums[9],nums[10]], m.css.z);
approx([nums[12],nums[13],nums[14]], [0,0,0]);

console.log('ALL TESTS PASSED (random cross-check x2000 + analytic poses + orthonormality + css map + matrix3d)');
