// orient.js — 设备朝向(DeviceOrientation) 角度 → 3D 姿态矩阵
// 约定严格遵循 W3C Device Orientation Events 规范 (https://www.w3.org/TR/orientation-event/)
//   地球坐标系(大地固定): X 向东, Y 指向真北, Z 垂直向上 (ENU)
//   设备坐标系:          x 屏幕向右(横握时沿长边), y 指向屏幕顶部, z 垂直屏幕向外
//   初始参考姿态(全 0):   设备平放于地面, 屏幕朝上, 顶部朝北
//   旋转顺序(内在旋转, 右手法则):
//     1) 绕地球 Z 轴旋转 α (方位/偏航, 0~360)
//     2) 绕 ① 之后的新 x 轴旋转 β (俯仰/前后倾, 理论上 -180~180, 手机一般 0~180)
//     3) 绕 ② 之后的新 y 轴旋转 γ (横滚/左右倾, -90~90)
// 返回: deviceToWorld 3x3 旋转矩阵(列向量 = 设备各轴在地球系中的单位方向),
//       以及投影到"CSS3D 变换空间"(x 右, y 向下, z 朝观察者, 即 CSS transform 原生坐标)
//       后的各轴方向: 这些列向量可直接作为 matrix3d 的列喂给元素, 并用于光照计算。
//
// 本文件是纯函数、无 DOM 依赖: 浏览器以 <script> 引入; Node 测试亦可直接加载。
// (浏览器的 deviceorientation 各轴数据可能带平台差异, 见 README「已知平台差异」)

(function () {
  'use strict';

  var DEG = Math.PI / 180;

  // Rodrigues 旋转公式: v 绕单位轴 k 旋转角度 theta(弧度)
  function rotateVec(v, k, theta) {
    var c = Math.cos(theta), s = Math.sin(theta), t = 1 - c;
    var kx = k[0], ky = k[1], kz = k[2];
    // 叉乘 k x v
    var cx = ky * v[2] - kz * v[1];
    var cy = kz * v[0] - kx * v[2];
    var cz = kx * v[1] - ky * v[0];
    var dot = kx * v[0] + ky * v[1] + kz * v[2];
    return [
      v[0] * c + cx * s + kx * dot * t,
      v[1] * c + cy * s + ky * dot * t,
      v[2] * c + cz * s + kz * dot * t
    ];
  }

  /**
   * 由 alpha/beta/gamma(度) 计算设备姿态。
   * @returns {{
   *   earth: { x:number[], y:number[], z:number[] },   // 设备轴在地球系 ENU 中的单位向量
   *   css:   { x:number[], y:number[], z:number[] },   // 同上, 投影到 CSS3D 变换空间(x右,y向下,z朝观察者)
   *   rowMajor9: number[]                               // css 空间旋转矩阵 3x3(行主序)
   * }}
   */
  function deviceFrameFromAngles(alphaDeg, betaDeg, gammaDeg) {
    var a = alphaDeg * DEG, b = betaDeg * DEG, g = gammaDeg * DEG;

    // 地球系基向量
    var E = [1, 0, 0], N = [0, 1, 0], U = [0, 0, 1];

    // ① 绕地球 Z(上) 轴转 α —— 初始设备系 == 地球系
    var ax = rotateVec(E, U, a);
    var ay = rotateVec(N, U, a);
    var az = rotateVec(U, U, a); // == U

    // ② 绕上一步的 x 轴(ax) 转 β
    var bx = ax;
    var by = rotateVec(ay, ax, b);
    var bz = rotateVec(az, ax, b);

    // ③ 绕上一步的 y 轴(by) 转 γ
    var dx = rotateVec(bx, by, g);
    var dy = by;
    var dz = rotateVec(bz, by, g);

    // 归一化防浮点漂移
    var nx = norm(dx), ny = norm(dy), nz = norm(dz);
    var ex = nx[0], ey = nx[1], ez = nx[2];
    var fx = ny[0], fy = ny[1], fz = ny[2];
    var gx = nz[0], gy = nz[1], gz = nz[2];

    // 地球空间列向量 = 设备轴
    var earth = {
      x: [ex, ey, ez],   // 设备 x(屏幕向右)
      y: [fx, fy, fz],   // 设备 y(顶部方向)
      z: [gx, gy, gz]    // 设备 z(屏幕法线, 向外)
    };

    // 投影到 CSS3D 变换空间: x=east, y=-up(向下), z=-north(北指向屏幕深处)
    // 注: CSS transform 的原生坐标系为 x 右 / y 向下 / z 朝观察者。
    function toCss(v) { return [v[0], -v[2], -v[1]]; }

    var css = {
      x: toCss(earth.x),
      y: toCss(earth.y),
      z: toCss(earth.z)
    };

    // css 空间旋转矩阵(行主序 3x3): M * [i] = css 坐标
    // 第 k 行 = css 轴与设备基向量的点积组合; 直接由列向量构造:
    //   M = [ colX | colY | colZ ], col 为设备轴在 css 中的坐标
    var c0 = css.x, c1 = css.y, c2 = css.z;
    var rowMajor9 = [
      c0[0], c1[0], c2[0],
      c0[1], c1[1], c2[1],
      c0[2], c1[2], c2[2]
    ];

    return { earth: earth, css: css, rowMajor9: rowMajor9 };
  }

  function norm(v) {
    var l = Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) || 1;
    return [v[0] / l, v[1] / l, v[2] / l];
  }

  // CSS transform: matrix3d(a1,b1,c1,d1, a2,b2,c2,d2, a3,b3,c3,d3, 0,0,0,1)
  // CSS 使用列主序; 4x4 = [ [c0 0] [c1 0] [c2 0] [tx ty tz 1] ]ᵀ 按列填。
  function cssMatrix3dFromCssAxes(cssAxes) {
    var c0 = cssAxes.x, c1 = cssAxes.y, c2 = cssAxes.z;
    return 'matrix3d(' + [
      c0[0], c0[1], c0[2], 0,
      c1[0], c1[1], c1[2], 0,
      c2[0], c2[1], c2[2], 0,
      0, 0, 0, 1
    ].join(',') + ')';
  }

  var api = {
    DEG: DEG,
    deviceFrameFromAngles: deviceFrameFromAngles,
    cssMatrix3dFromCssAxes: cssMatrix3dFromCssAxes
  };

  if (typeof window !== 'undefined') window.__gyroOrient = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})();
