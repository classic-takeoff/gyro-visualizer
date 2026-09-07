// orient.js — 设备朝向(DeviceOrientation) 角度 → 3D 姿态矩阵 + 矩阵工具
// 约定严格遵循 W3C Device Orientation Events 规范 (https://www.w3.org/TR/orientation-event/)
//   地球坐标系(大地固定): X 向东, Y 指向真北, Z 垂直向上 (ENU)
//   设备坐标系:          x 屏幕向右(横握时沿长边), y 指向屏幕顶部, z 垂直屏幕向外
//   初始参考姿态(全 0):   设备平放于地面, 屏幕朝上, 顶部朝北
//   旋转顺序(内在旋转, 右手法则):
//     1) 绕地球 Z 轴旋转 α (方位/偏航, 0~360)
//     2) 绕 ① 之后的新 x 轴旋转 β (俯仰/前后倾, 理论 -180~180, 手机一般 0~180)
//     3) 绕 ② 之后的新 y 轴旋转 γ (横滚/左右倾, -90~90)
// 返回: deviceToWorld 3x3 旋转矩阵(列向量 = 设备各轴在地球系中的单位方向),
//       以及投影到"CSS3D 变换空间"(x 右, y 向下, z 朝观察者, 即 CSS transform 原生坐标)
//       后的各轴方向: 这些列向量可直接作为 matrix3d 的列喂给元素, 并用于光照计算。
//
// 说明: 真机浏览器对 Euler 角的基准/符号存在平台差异(部分机型的传感器"自然方向"为横屏等),
//       因此页面提供"对准我/±90°/重置"校准按钮 —— 校准 = 乘一个固定旋转 X:
//       显示姿态列 = X * 原始列。本文件的 mat3 工具即用于构造 X。
//
// 本文件是纯函数、无 DOM 依赖: 浏览器以 <script> 引入; Node 测试亦可直接加载。

(function () {
  'use strict';

  var DEG = Math.PI / 180;

  // Rodrigues 旋转公式: v 绕单位轴 k 旋转角度 theta(弧度)
  function rotateVec(v, k, theta) {
    var c = Math.cos(theta), s = Math.sin(theta), t = 1 - c;
    var kx = k[0], ky = k[1], kz = k[2];
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

  function norm(v) {
    var l = Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) || 1;
    return [v[0] / l, v[1] / l, v[2] / l];
  }

  /**
   * 由 alpha/beta/gamma(度) 计算设备姿态。
   * @returns {{
   *   earth: { x, y, z },   // 设备轴在地球系 ENU 中的单位向量
   *   css:   { x, y, z },   // 投影到 CSS3D 变换空间(x右,y向下,z朝观察者)
   *   rowMajor9: number[]
   * }}
   */
  function deviceFrameFromAngles(alphaDeg, betaDeg, gammaDeg) {
    var a = alphaDeg * DEG, b = betaDeg * DEG, g = gammaDeg * DEG;
    var E = [1, 0, 0], N = [0, 1, 0], U = [0, 0, 1];

    // ① 绕地球 Z(上) 转 α(初始设备系 == 地球系)
    var ax = rotateVec(E, U, a);
    var ay = rotateVec(N, U, a);
    var az = rotateVec(U, U, a);
    // ② 绕上一步 x 轴转 β
    var by = rotateVec(ay, ax, b);
    var bz = rotateVec(az, ax, b);
    // ③ 绕上一步 y 轴转 γ
    var dx = rotateVec(ax, by, g);
    var dy = by;
    var dz = rotateVec(bz, by, g);

    var nx = norm(dx), ny = norm(dy), nz = norm(dz);
    var earth = { x: nx, y: ny, z: nz };

    // 投影到 CSS3D 空间: x=east, y=-up, z=-north(CSS 原生坐标 y 向下、z 朝观察者)
    function toCss(v) { return [v[0], -v[2], -v[1]]; }
    var css = { x: toCss(nx), y: toCss(ny), z: toCss(nz) };

    var c0 = css.x, c1 = css.y, c2 = css.z;
    return {
      earth: earth, css: css,
      rowMajor9: [
        c0[0], c1[0], c2[0],
        c0[1], c1[1], c2[1],
        c0[2], c1[2], c2[2]
      ]
    };
  }

  // ================= 3x3 矩阵工具(行主序, 用于校准 X) =================

  function identityMat3() {
    return [1, 0, 0, 0, 1, 0, 0, 0, 1];
  }
  // 由列向量组 {x,y,z} 构造行主序矩阵(列 i = css 轴 i 的图像)
  function mat3FromCols(cols) {
    return [
      cols.x[0], cols.y[0], cols.z[0],
      cols.x[1], cols.y[1], cols.z[1],
      cols.x[2], cols.y[2], cols.z[2]
    ];
  }
  function mat3Transpose(m) {
    return [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]];
  }
  function mat3Mul3(a, b) {
    var r = new Array(9);
    for (var i = 0; i < 3; i++) {
      for (var j = 0; j < 3; j++) {
        r[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
      }
    }
    return r;
  }
  // 行主序矩阵 × 列向量组 → 新的列向量组
  function mat3ApplyToCols(m, cols) {
    function ap(c) {
      return [
        m[0] * c[0] + m[1] * c[1] + m[2] * c[2],
        m[3] * c[0] + m[4] * c[1] + m[5] * c[2],
        m[6] * c[0] + m[7] * c[1] + m[8] * c[2]
      ];
    }
    return { x: ap(cols.x), y: ap(cols.y), z: ap(cols.z) };
  }
  // 绕单位轴 u(世界坐标) 旋转 thetaDeg 的旋转矩阵(行主序), 用 Rodrigues 生成列再转置
  function rotMat3Axis(u, thetaDeg) {
    var th = thetaDeg * DEG;
    var ex = rotateVec([1, 0, 0], u, th);
    var ey = rotateVec([0, 1, 0], u, th);
    var ez = rotateVec([0, 0, 1], u, th);
    // 列主序转行主序: 行 i = [ex[i], ey[i], ez[i]]
    return [ex[0], ey[0], ez[0], ex[1], ey[1], ez[1], ex[2], ey[2], ez[2]];
  }

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
    cssMatrix3dFromCssAxes: cssMatrix3dFromCssAxes,
    identityMat3: identityMat3,
    mat3FromCols: mat3FromCols,
    mat3Transpose: mat3Transpose,
    mat3Mul3: mat3Mul3,
    mat3ApplyToCols: mat3ApplyToCols,
    rotMat3Axis: rotMat3Axis,
    rotateVec: rotateVec,
    norm: norm
  };

  if (typeof window !== 'undefined') window.__gyroOrient = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})();
