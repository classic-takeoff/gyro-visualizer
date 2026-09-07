/* main.js — 陀螺仪姿态可视化主逻辑(经典脚本)
 *
 * 设计(按真实体验需求):
 *  1) 单一数据源: 优先 deviceorientationabsolute, 无则 deviceorientation;
 *     绝不混用两种(它们的 alpha 基准不同, 交叉触发会互相跳→“抖动”元凶之一)。
 *  2) 自动锚定(无按钮): 传感器开启后取“最平稳时刻”的姿态作基准,
 *     并把该基准显示为“竖立、屏幕正对观察者”; 之后模型与角度 = 相对基准的增量。
 *     这样无论设备坐标基准如何(部分机型自然方向为横屏等), 你"怎么拿就怎么显示",
 *     双击 3D 画面可随时用当前拿法重新锚定。
 *  3) 指数平滑(α 解卷绕处理 359→0 跳变), 固定中档, 无 UI。
 */
(function () {
  'use strict';

  var O = window.__gyroOrient || {};
  var deviceFrame = O.deviceFrameFromAngles;
  if (!deviceFrame) { console.error('orient.js 未加载'); return; }

  // ---------- 小工具 ----------
  function $(id) { return document.getElementById(id); }
  function clamp(v, a, b) { return v < a ? a : (v > b ? b : v); }
  function wrap360(x) { x = x % 360; return x < 0 ? x + 360 : x; }
  function fmt(v, d) { return v.toFixed(d == null ? 1 : d); }

  var MOBILE = (typeof matchMedia === 'function' && matchMedia('(pointer:coarse)').matches) ||
               /iPhone|iPad|iPod|Android|Mobile/i.test(navigator.userAgent || '');
  var NEED_GESTURE = 'DeviceOrientationEvent' in window &&
                     typeof window.DeviceOrientationEvent.requestPermission === 'function';
  var SENSOR_SUPPORTED = 'DeviceOrientationEvent' in window;

  // ---------- DOM ----------
  var el = {
    dotLive: $('dotLive'), chipMode: $('chipMode'), chipHz: $('chipHz'), chipAbs: $('chipAbs'),
    phone: $('phone'), axismodel: $('axismodel'), orbit: $('orbit'), scene: $('scene'),
    shadeFront: document.querySelector('#phone .f-front .shade'),
    shadeBack: document.querySelector('#phone .f-back .shade'),
    shadeRight: document.querySelector('#phone .f-right .shade'),
    shadeLeft: document.querySelector('#phone .f-left .shade'),
    shadeTop: document.querySelector('#phone .f-top .shade'),
    shadeBottom: document.querySelector('#phone .f-bottom .shade'),
    vA: $('vA'), vB: $('vB'), vG: $('vG'),
    bvA: $('bvA'), bvB: $('bvB'), bvG: $('bvG'),
    fillA: $('fillA'), fillB: $('fillB'), fillG: $('fillG'),
    poseLine: $('poseLine'),
    msgSensor: $('msgSensor'), msgSensorText: $('msgSensorText'),
    msgError: $('msgError'), msgErrorText: $('msgErrorText'), btnEnable: $('btnEnable'),
    simA: $('simA'), simB: $('simB'), simG: $('simG'),
    simAv: $('simAv'), simBv: $('simBv'), simGv: $('simGv'),
    btnDemo: $('btnDemo'), btnZero: $('btnZero'),
    chart: $('chart'), chartPause: $('chartPause'), segMode: $('segMode'),
    helpOv: $('helpOv')
  };

  // ---------- 状态 ----------
  var S = {
    mode: 'sim',
    sensorState: 'idle',
    useAbsolute: null,           // null 未定 / true 用绝对流 / false 用相对流
    // 显示角度(平滑后, 相对锚定基准): α 连续(uw), β,γ
    uw: 0, db: 90, dg: 0, dispValid: false,
    // 目标(未平滑的锚定后角度, 需从矩阵解出)
    tA: 0, tB: 90, tG: 0,
    solvePrev: { a: 0, b: 90, g: 0 },
    // 锚定
    anchor: null,                // X 矩阵(行主序9); null=未锚定(显示基准姿态)
    anchorCollect: null,
    rawPrev: null,               // 上一帧原始 css 列(用于抖动抑制的双重平滑, 可选)
    smoothK: 0.35,               // 固定中档平滑系数参考(实际按时间常数)
    demo: false, demoStart: 0,
    lastSolveT: 0,
    chartPaused: false,
    chart: { t: [], a: [], b: [], g: [] },
    lastSample: 0, lastTick: 0,
    dirty: true, needsSolve: false,
    evCount: 0, evWinStart: performance.now(), lastEvT: 0
  };
  var CAM = { pitch: -8, yaw: 14, zoom: -6 };
  var TARGET = deviceFrame(0, 90, 0).css;   // 基准姿态 = 竖立正对观察者(矩阵=显示恒等)

  // ---------- 求解: css 列矩阵 → 显示欧拉角 (9分量 Gauss-Newton, 依连续性取支) ----------
  function flatCols(c) {
    return [c.x[0], c.x[1], c.x[2], c.y[0], c.y[1], c.y[2], c.z[0], c.z[1], c.z[2]];
  }
  function eulerFromCols(targetCols, guess) {
    var tgt = flatCols(targetCols);
    var a = guess.a, b = guess.b, g = guess.g;
    var eps = 1e-4, step = 0.7;
    for (var it = 0; it < 24; it++) {
      var cur = flatCols(deviceFrame(a, b, g).css);
      var r = new Array(9);
      for (var k = 0; k < 9; k++) r[k] = tgt[k] - cur[k];
      // 数值雅可比 9x3
      var Ja = flatCols(deviceFrame(a + eps, b, g).css);
      var Jb = flatCols(deviceFrame(a, b + eps, g).css);
      var Jg = flatCols(deviceFrame(a, b, g + eps).css);
      var JtJ = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], Jtr = [0, 0, 0];
      for (var i = 0; i < 9; i++) {
        var col = [ (Ja[i] - cur[i]) / eps, (Jb[i] - cur[i]) / eps, (Jg[i] - cur[i]) / eps ];
        for (var m = 0; m < 3; m++) {
          Jtr[m] += col[m] * r[i];
          for (var n = 0; n < 3; n++) JtJ[m][n] += col[m] * col[n];
        }
      }
      var d = solve3(JtJ, Jtr);
      var da = ((wrap360(a + d[0]) - a + 540) % 360 - 180) * step;
      var db = d[1] * step, dg = d[2] * step;
      a = wrap360(a + da); b = clamp(b + db, -180, 180); g = clamp(g + dg, -180, 180);
      if (Math.max(Math.abs(da), Math.abs(db), Math.abs(dg)) < 1e-5) break;
    }
    return { a: wrap360(a), b: clamp(b, -180, 180), g: clamp(g, -180, 180) };
  }
  function solve3(A, bvec) {
    // 高斯消元 3x3
    var M = [A[0][0], A[0][1], A[0][2], A[1][0], A[1][1], A[1][2], A[2][0], A[2][1], A[2][2]];
    var v = [bvec[0], bvec[1], bvec[2]];
    for (var i = 0; i < 3; i++) {
      var piv = i;
      for (var k = i + 1; k < 3; k++) if (Math.abs(M[k * 3 + i]) > Math.abs(M[piv * 3 + i])) piv = k;
      if (piv !== i) {
        for (var k2 = 0; k2 < 3; k2++) { var tmp = M[i * 3 + k2]; M[i * 3 + k2] = M[piv * 3 + k2]; M[piv * 3 + k2] = tmp; }
        var t2 = v[i]; v[i] = v[piv]; v[piv] = t2;
      }
      var d0 = M[i * 3 + i];
      if (Math.abs(d0) < 1e-12) { v[i] = 0; continue; }
      for (var j = i + 1; j < 3; j++) {
        var f = M[j * 3 + i] / d0;
        for (var k3 = i; k3 < 3; k3++) M[j * 3 + k3] -= f * M[i * 3 + k3];
        v[j] -= f * v[i];
      }
    }
    var x = [0, 0, 0];
    for (var m = 2; m >= 0; m--) {
      var s = v[m];
      for (var n = m + 1; n < 3; n++) s -= M[m * 3 + n] * x[n];
      x[m] = M[m * 3 + m] ? s / M[m * 3 + m] : 0;
    }
    return x;
  }

  // ---------- 传感器: 单一数据源 ----------
  function useThisEvent(e) {
    if (e.absolute) {
      S.useAbsolute = true;
      return true;
    }
    // 相对事件: 若已见绝对流则忽略; 否则若绝对流长时间无事件也用相对
    if (S.useAbsolute === true) return false;
    if (S.useAbsolute === null) {
      S.useAbsolute = false;
    }
    return true;
  }
  function onOrient(e) {
    var a = e.alpha, b = e.beta, g = e.gamma;
    if (a == null || b == null || g == null) return;
    if (!isFinite(a + b + g)) return;
    if (!useThisEvent(e)) return;
    S.evCount++;
    S.lastEvT = performance.now();
    S.lastRaw = { a: wrap360(a), b: clamp(b, -180, 180), g: clamp(g, -180, 180) };
    handleRaw(S.lastRaw.a, S.lastRaw.b, S.lastRaw.g);
  }
  function attachSensor() {
    // 先只挂普通事件; 若浏览器声明支持绝对事件则也挂, 以绝对流为准
    window.addEventListener('deviceorientation', onOrient, true);
    if ('ondeviceorientationabsolute' in window) window.addEventListener('deviceorientationabsolute', onOrient, true);
    return true;
  }
  function detachSensor() {
    window.removeEventListener('deviceorientation', onOrient, true);
    window.removeEventListener('deviceorientationabsolute', onOrient, true);
  }

  // ---------- 原始角度 → 锚定/增量 → 目标显示角 ----------
  function handleRaw(a, b, g) {
    var cols = deviceFrame(a, b, g).css;
    if (!S.anchor) {
      collectAnchor(a, b, g);
      return;
    }
    // 节流: 30Hz 解算足够(显示侧还有平滑)
    var now2 = performance.now();
    if (now2 - S.lastSolveT < 30) return;
    S.lastSolveT = now2;
    // 校正到基准系
    var D = O.mat3ApplyToCols(S.anchor, cols);
    var sol = eulerFromCols(D, S.solvePrev);
    S.solvePrev = sol;
    S.tA = sol.a; S.tB = sol.b; S.tG = sol.g;
    S.needsSolve = true;
  }
  var anchorBuf = [];
  var anchorT0 = 0;
  function collectAnchor(a, b, g) {
    var now = performance.now();
    if (!anchorT0) anchorT0 = now;
    anchorBuf.push({ t: now, a: a, b: b, g: g });
    // 清理 >1.6s
    while (anchorBuf.length && now - anchorBuf[0].t > 1600) anchorBuf.shift();
    if (now - anchorT0 < 350) return;          // 前 0.35s 先不判定
    // 选“最平稳”: 与其邻居的欧氏距离和最小者作为中心
    var best = null, bestScore = Infinity;
    for (var i = 1; i < anchorBuf.length - 1; i++) {
      var p = anchorBuf[i];
      var score = 0;
      for (var j = -2; j <= 2; j++) {
        var q = anchorBuf[i + j];
        if (!q) continue;
        var da = ((q.a - p.a + 540) % 360) - 180;
        score += da * da + (q.b - p.b) * (q.b - p.b) + (q.g - p.g) * (q.g - p.g);
      }
      if (score < bestScore) { bestScore = score; best = p; }
    }
    var stableEnough = anchorBuf.length >= 8 && bestScore < 2.2;
    var timeUp = now - anchorT0 > 1500;
    if (!best || (!stableEnough && !timeUp)) return;
    // 锚定: 把此刻的原始姿态映射为基准姿态
    var bcols = deviceFrame(best.a, best.b, best.g).css;
    S.anchor = O.mat3Mul3(O.mat3FromCols(TARGET), O.mat3Transpose(O.mat3FromCols(bcols)));
    S.solvePrev = { a: 0, b: 90, g: 0 };
    anchorBuf = [];
    S.lastSolveT = 0;
    syncAbsLabel();
    updateSceneMsg();
    if (S.sensorState === 'running') setSensorState('running');
  }
  function reAnchorNow() {
    // 双击: 用当前最新平滑后的目标角对应矩阵? 用当前原始帧更准: 直接以最近一次原始值为准
    if (S.lastRaw) {
      var cols = deviceFrame(S.lastRaw.a, S.lastRaw.b, S.lastRaw.g).css;
      S.anchor = O.mat3Mul3(O.mat3FromCols(TARGET), O.mat3Transpose(O.mat3FromCols(cols)));
    } else {
      S.anchor = O.identityMat3();
    }
    S.solvePrev = { a: 0, b: 90, g: 0 };
    S.needsSolve = true;
    syncAbsLabel();
    updateSceneMsg();
  }
  function ingestRawStore(a, b, g) {
    S.lastRaw = { a: a, b: b, g: g };
  }

  // ---------- 平滑 + 显示 ----------
  function stepSmooth() {
    if (S.needsSolve && S.mode === 'sensor') {
      S.needsSolve = false;
      // 时间常数: α 更稳, β/γ 适中
      var dt = clamp((performance.now() - (S.lastTick || performance.now())) / 1000, 0.005, 0.1);
      var kA = 1 - Math.exp(-dt / 0.16);
      var kBG = 1 - Math.exp(-dt / 0.08);
      if (!S.dispValid) {
        S.uw = S.tA; S.db = S.tB; S.dg = S.tG;
        S.dispValid = true;
        S.dirty = true;
        return;
      }
      var cur = wrap360(S.uw);
      var diff = ((S.tA - cur + 540) % 360) - 180;
      var dUw = diff * kA;
      var nB = S.db + (S.tB - S.db) * kBG;
      var nG = S.dg + (S.tG - S.dg) * kBG;
      if (Math.abs(dUw) > 0.003 || Math.abs(nB - S.db) > 0.003 || Math.abs(nG - S.dg) > 0.003) S.dirty = true;
      S.uw += dUw; S.db = nB; S.dg = nG;
    }
  }
  function display() {
    return { a: wrap360(S.uw), b: clamp(S.db, -180, 180), g: clamp(S.dg, -180, 180) };
  }
  // 模拟/演示直通
  function ingestSim(a, b, g) {
    S.uw = wrap360(a); S.db = clamp(b, -180, 180); S.dg = clamp(g, -180, 180);
    S.dispValid = true;
    S.dirty = true;
  }

  // ---------- 模式 ----------
  function setSensorState(st) {
    S.sensorState = st;
    if (st === 'running') el.dotLive.className = 'dot on';
    else if (st === 'lost') el.dotLive.className = 'dot warn';
    else el.dotLive.className = 'dot';
    if (S.mode !== 'sensor') { syncChip(); return; }
    el.msgError.classList.remove('show');
    el.msgSensor.classList.remove('show');
    el.btnEnable.style.display = 'none';
    switch (st) {
      case 'running':
        el.chipMode.textContent = S.anchor ? '传感器·运行中' : '传感器·对准中…';
        updateSceneMsg();
        break;
      case 'wait':
        el.chipMode.textContent = '传感器·待授权';
        el.msgSensorText.textContent = 'iOS 需要授权：点击「传感器」后在弹窗选“允许”。授权后请按你平时拿手机的姿势拿着，稍候模型即自动对准。';
        el.msgSensor.classList.add('show');
        updateSceneMsg();
        break;
      case 'denied':
        el.chipMode.textContent = '传感器·已拒绝';
        el.msgErrorText.textContent = '传感器权限被拒绝：页面需 HTTPS 或 localhost；iOS 需在 Safari 中允许“运动与健身”。可切回模拟模式。';
        el.msgError.classList.add('show');
        break;
      case 'lost':
        el.dotLive.className = 'dot warn';
        el.chipMode.textContent = '传感器·无数据';
        el.btnEnable.style.display = '';
        el.msgErrorText.textContent = '传感器一直没有数据。可点击重试，或切换模拟模式。';
        el.msgError.classList.add('show');
        break;
      default: el.chipMode.textContent = '传感器'; updateSceneMsg(); break;
    }
  }
  function updateSceneMsg() {
    var m = $('sceneMsg');
    if (!m) return;
    if (S.mode !== 'sensor') {
      m.textContent = '模拟模式：用右侧滑杆或“演示摇摆”驱动模型';
      return;
    }
    if (!SENSOR_SUPPORTED || S.sensorState === 'denied' || S.sensorState === 'unsupported' || S.sensorState === 'lost') {
      m.textContent = '传感器暂不可用，可切“模拟”先体验';
    } else if (S.sensorState === 'wait') {
      m.textContent = '点击右上角「传感器」并在弹窗允许后开始';
    } else if (!S.anchor) {
      m.textContent = '正在对准… 请像平时看手机那样竖握并拿稳（约 1 秒）';
    } else {
      m.textContent = '基准 = 当前拿姿 · 之后转动即随之变化 · 双击画面可用新拿姿重新对准';
    }
  }
  function showUnsupported() {
    el.chipMode.textContent = '模拟模式 · 传感器不可用';
    updateSceneMsg();
    el.msgSensor.classList.remove('show');
    el.msgError.classList.remove('show');
    el.msgErrorText.textContent = '当前浏览器不支持 DeviceOrientation，或不在安全上下文（需 HTTPS/localhost）。已自动使用模拟模式。';
    el.msgError.classList.add('show');
  }
  function paintSeg() {
    document.querySelectorAll('#segMode button').forEach(function (b) {
      b.classList.toggle('active', b.dataset.mode === S.mode);
    });
  }
  function setMode(m, fromGesture) {
    if (m === S.mode) { if (m === 'sensor') tryEnableSensor(fromGesture); return; }
    S.mode = m;
    paintSeg();
    if (m === 'sensor') {
      resetSensorSession();
      tryEnableSensor(fromGesture);
    } else {
      detachSensor();
      resetSensorSession();
      el.dotLive.className = 'dot';
      el.msgSensor.classList.remove('show');
      el.msgError.classList.remove('show');
      el.btnEnable.style.display = 'none';
      el.chipMode.textContent = '模拟模式';
      syncChip();
      syncAbsLabel();
      updateSceneMsg();
    }
  }
  function resetSensorSession() {
    S.anchor = null; S.useAbsolute = null; S.dispValid = false;
    S.solvePrev = { a: 0, b: 90, g: 0 };
    S.tA = 0; S.tB = 90; S.tG = 0;
    anchorBuf = []; anchorT0 = 0; S.lastRaw = null;
    S.uw = 0; S.db = 90; S.dg = 0;
    S.dirty = true;
    updateSceneMsg();
  }
  function tryEnableSensor(fromGesture) {
    if (S.mode !== 'sensor') return;
    if (!SENSOR_SUPPORTED) { showUnsupported(); return; }
    if (NEED_GESTURE) {
      if (!fromGesture) { setSensorState('wait'); return; }
      setSensorState('wait');
      try {
        var p = window.DeviceOrientationEvent.requestPermission();
        if (p && typeof p.then === 'function') {
          p.then(function (res) {
            if (res === 'granted') { attachSensor(); setSensorState('running'); }
            else setSensorState('denied');
          }).catch(function () { setSensorState('denied'); });
        } else { attachSensor(); setSensorState('running'); }
      } catch (err) { setSensorState('denied'); }
    } else { attachSensor(); setSensorState('running'); }
  }

  // ---------- 渲染 ----------
  function render() {
    var ang = display();
    var frame = deviceFrame(ang.a, ang.b, ang.g);
    if (!frame) return;
    // 非传感器/未锚定时仍以当前显示角直接画(模拟模式即如此)
    var F = frame.css;
    if (S.mode === 'sim' || !S.anchor) {
      // 模拟: 显示角即(0..360/±180) 语义, 无需额外变换
    }
    var m3 = 'translate(-50%,-50%) ' + O.cssMatrix3dFromCssAxes(F);
    el.phone.style.transform = m3;
    el.axismodel.style.transform = m3;

    var L = O.norm([0.55, -0.5, 0.85]);
    var faces = [
      { e: el.shadeFront, n: F.z, d: 1 }, { e: el.shadeBack, n: F.z, d: -1 },
      { e: el.shadeRight, n: F.x, d: 1 }, { e: el.shadeLeft, n: F.x, d: -1 },
      { e: el.shadeTop, n: F.y, d: 1 }, { e: el.shadeBottom, n: F.y, d: -1 }
    ];
    for (var i = 0; i < faces.length; i++) {
      var nd = dot(faces[i].n, L) * faces[i].d;
      faces[i].e.style.opacity = clamp(0.72 - 0.72 * Math.max(0, nd), 0, 0.8).toFixed(3);
    }
    function dot(v, w) { return v[0] * w[0] + v[1] * w[1] + v[2] * w[2]; }

    renderNumbers(ang);
    renderBars(ang);
    renderPoseText(ang);
    sampleChart();
  }
  function renderNumbers(ang) {
    el.vA.textContent = fmt(ang.a); el.vB.textContent = fmt(ang.b); el.vG.textContent = fmt(ang.g);
    el.bvA.textContent = fmt(ang.a) + '°'; el.bvB.textContent = fmt(ang.b) + '°'; el.bvG.textContent = fmt(ang.g) + '°';
    el.simAv.textContent = fmt(parseFloat(el.simA.value)) + '°';
    el.simBv.textContent = fmt(parseFloat(el.simB.value)) + '°';
    el.simGv.textContent = fmt(parseFloat(el.simG.value)) + '°';
  }
  function renderBars(ang) {
    el.fillA.style.width = clamp(ang.a / 360 * 100, 0, 100) + '%';
    el.fillB.style.width = clamp(ang.b / 180 * 100, 0, 100) + '%';
    var gp = ang.g / 180 * 100;
    if (ang.g >= 0) { el.fillG.style.left = '50%'; el.fillG.style.width = clamp(gp, 0, 100) + '%'; }
    else { el.fillG.style.left = (50 + gp) + '%'; el.fillG.style.width = clamp(-gp, 0, 100) + '%'; }
  }
  function renderPoseText(ang) {
    var b = ang.b, g = ang.g, tags = [];
    if (b < 20) tags.push({ t: '接近平放', c: 'var(--a)' });
    else if (b > 160) tags.push({ t: '接近翻转', c: 'var(--g)' });
    else if (b >= 70 && b <= 110) tags.push({ t: '竖立', c: 'var(--ok)' });
    else tags.push({ t: '倾斜', c: 'var(--b)' });
    if (g > 12) tags.push({ t: '右倾', c: 'var(--g)' });
    else if (g < -12) tags.push({ t: '左倾', c: 'var(--g)' });
    else tags.push({ t: '左右端正', c: 'var(--b)' });
    el.poseLine.innerHTML = '';
    tags.forEach(function (tg) {
      var s = document.createElement('span');
      s.className = 'tag';
      s.style.borderColor = tg.c; s.style.color = tg.c;
      s.textContent = tg.t;
      el.poseLine.appendChild(s);
    });
  }

  // ---------- 曲线 ----------
  function sampleChart() {
    var now = performance.now();
    if (S.chartPaused) return;
    if (now - S.lastSample < 33) return;
    S.lastSample = now;
    var ang = display();
    var c = S.chart;
    c.t.push(now); c.a.push(ang.a); c.b.push(ang.b); c.g.push(ang.g);
    var span = 30000;
    while (c.t.length && now - c.t[0] > span) { c.t.shift(); c.a.shift(); c.b.shift(); c.g.shift(); }
  }
  function drawChart() {
    var cv = el.chart, wrap = $('chartWrap');
    var w = wrap.clientWidth, h = wrap.clientHeight;
    if (w < 10) return;
    var dpr = window.devicePixelRatio || 1;
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) {
      cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr);
    }
    var ctx = cv.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    var c = S.chart;
    if (!c.t.length) {
      ctx.fillStyle = 'rgba(255,255,255,.14)'; ctx.font = '11px sans-serif';
      ctx.textAlign = 'center'; ctx.fillText('等待数据…', w / 2, h / 2); ctx.textAlign = 'left';
      return;
    }
    var padL = 6, padR = 8, padT = 4, padB = 6;
    var lane = (h - padT - padB) / 3;
    var series = [
      { d: c.a, min: 0, max: 360, col: '#4cc9f0' },
      { d: c.b, min: -180, max: 180, col: '#ffb84d' },
      { d: c.g, min: -180, max: 180, col: '#ff6b9d' }
    ];
    var t0 = c.t[0], t1 = c.t[c.t.length - 1];
    var tspan = Math.max(t1 - t0, 1000);
    function xAt(t) { return padL + (w - padL - padR) * (1 - (t1 - t) / tspan); }
    for (var s = 0; s < 3; s++) {
      var y0 = padT + lane * s, y1 = y0 + lane;
      ctx.strokeStyle = 'rgba(255,255,255,.07)';
      ctx.beginPath();
      ctx.moveTo(padL, y1); ctx.lineTo(w - padR, y1);
      ctx.moveTo(padL, y0 + lane / 2); ctx.lineTo(w - padR, y0 + lane / 2);
      ctx.stroke();
      var ser = series[s];
      ctx.strokeStyle = ser.col; ctx.lineWidth = 1.6;
      ctx.beginPath();
      var started = false;
      var step = Math.max(1, Math.floor(c.t.length / (w * 2)));
      for (var i = 0; i < c.t.length; i += step) {
        var v = ser.d[i];
        var px = xAt(c.t[i]);
        var py = y1 - 5 - ((v - ser.min) / (ser.max - ser.min)) * (lane - 10);
        if (!started) { ctx.moveTo(px, py); started = true; }
        else ctx.lineTo(px, py);
      }
      ctx.stroke();
      var last = ser.d[ser.d.length - 1];
      ctx.fillStyle = ser.col;
      ctx.beginPath(); ctx.arc(xAt(t1), y1 - 5 - ((last - ser.min) / (ser.max - ser.min)) * (lane - 10), 2, 0, Math.PI * 2); ctx.fill();
    }
  }

  // ---------- 视角 ----------
  function applyCam() {
    el.orbit.style.transform = 'rotateX(' + CAM.pitch + 'deg) rotateY(' + CAM.yaw + 'deg) translateZ(' + CAM.zoom + 'px)';
  }
  function bindCam() {
    var drag = null, taps = 0, tapT = 0;
    function onDown(e) {
      drag = { x: e.clientX, y: e.clientY, moved: false };
      try { el.scene.setPointerCapture(e.pointerId); } catch (err) {}
      el.scene.classList.add('dragging');
    }
    function onMove(e) {
      if (!drag) return;
      var dx = e.clientX - drag.x, dy = e.clientY - drag.y;
      if (Math.abs(dx) + Math.abs(dy) > 3) drag.moved = true;
      CAM.yaw += dx * 0.32;
      CAM.pitch = clamp(CAM.pitch - dy * 0.32, -88, 45);
      drag.x = e.clientX; drag.y = e.clientY;
      applyCam();
    }
    function onUp(e) {
      var wasTap = drag && !drag.moved;
      drag = null;
      el.scene.classList.remove('dragging');
      try { el.scene.releasePointerCapture(e.pointerId); } catch (err) {}
      if (wasTap) {
        var now = performance.now();
        if (now - tapT < 350 && ++taps >= 2) {
          taps = 0;
          // 双击: 用当前拿法重新锚定(传感器模式)
          if (S.mode === 'sensor' && S.anchor) { reAnchorNow(); }
          else camReset();
        } else {
          if (now - tapT >= 350) taps = 1;
          tapT = now;
        }
      }
    }
    el.scene.addEventListener('pointerdown', onDown);
    el.scene.addEventListener('pointermove', onMove);
    el.scene.addEventListener('pointerup', onUp);
    el.scene.addEventListener('pointercancel', function () { drag = null; el.scene.classList.remove('dragging'); });
    el.scene.addEventListener('wheel', function (e) {
      e.preventDefault();
      CAM.zoom = clamp(CAM.zoom + e.deltaY * 0.6, -850, 460);
      applyCam();
    }, { passive: false });
  }
  function camReset() { CAM.pitch = -8; CAM.yaw = 14; CAM.zoom = -6; applyCam(); }

  // ---------- 模拟 ----------
  function stopDemo() {
    if (!S.demo) return;
    S.demo = false;
    el.btnDemo.textContent = '▶ 演示摇摆';
  }
  function bindSim() {
    [el.simA, el.simB, el.simG].forEach(function (inp) {
      inp.addEventListener('input', function () {
        if (S.demo) stopDemo();
        if (S.mode !== 'sim') setMode('sim');
        ingestSim(parseFloat(el.simA.value), parseFloat(el.simB.value), parseFloat(el.simG.value));
      });
    });
    el.btnZero.addEventListener('click', function () {
      stopDemo();
      el.simA.value = 0; el.simB.value = 0; el.simG.value = 0;
      if (S.mode !== 'sim') setMode('sim');
      ingestSim(0, 0, 0);
    });
    el.btnDemo.addEventListener('click', function () {
      if (S.demo) { stopDemo(); return; }
      if (S.mode !== 'sim') setMode('sim');
      S.demo = true; S.demoStart = performance.now();
      el.btnDemo.textContent = '⏸ 停止演示';
    });
  }

  // ---------- 其它 UI ----------
  function syncAbsLabel() {
    if (S.mode !== 'sensor') el.chipAbs.textContent = '—';
    else el.chipAbs.textContent = S.anchor ? '起始基准' : '对准中…';
  }
  function syncChip() {
    syncAbsLabel();
    if (S.mode === 'sensor' && S.sensorState === 'running') el.dotLive.className = 'dot on';
  }
  function bindUI() {
    el.segMode.addEventListener('click', function (e) {
      var b = e.target.closest('button');
      if (!b) return;
      if (S.demo) stopDemo();
      setMode(b.dataset.mode, true);
    });
    $('btnHelp').addEventListener('click', function () { el.helpOv.classList.add('show'); });
    $('btnHelpClose').addEventListener('click', function () { el.helpOv.classList.remove('show'); });
    el.helpOv.addEventListener('click', function (e) { if (e.target === el.helpOv) el.helpOv.classList.remove('show'); });
    el.btnEnable.addEventListener('click', function () { tryEnableSensor(true); });
    el.chartPause.addEventListener('click', function () {
      S.chartPaused = !S.chartPaused;
      el.chartPause.textContent = S.chartPaused ? '▶' : '⏸';
    });
    document.addEventListener('keydown', function (e) { if (e.key === 'Escape') el.helpOv.classList.remove('show'); });
  }
  function hzLoop() {
    var now = performance.now();
    if (now - S.evWinStart >= 1000) {
      S.evCount = Math.round(S.evCount * 1000 / (now - S.evWinStart));
      el.chipHz.textContent = S.evCount + ' Hz';
      S.evCount = 0; S.evWinStart = now;
      if (S.mode === 'sensor' && S.sensorState === 'running' && now - S.lastEvT > 2500) setSensorState('lost');
    }
    setTimeout(hzLoop, 250);
  }
  function bindResize() {
    function sizePhone() {
      var w = el.scene.clientWidth, h = el.scene.clientHeight;
      var pw = clamp(Math.min(w * 0.32, h * 0.46), 54, 180);
      var ph = pw * 2.15, pd = pw * 0.135;
      var st = { '--pw': pw.toFixed(1) + 'px', '--ph': ph.toFixed(1) + 'px', '--pd': pd.toFixed(1) + 'px' };
      for (var k in st) { el.phone.style.setProperty(k, st[k]); el.axismodel.style.setProperty(k, st[k]); }
    }
    if ('ResizeObserver' in window) {
      var ro = new ResizeObserver(function () { sizePhone(); drawChart(); });
      ro.observe(el.scene); ro.observe($('chartWrap'));
    } else {
      window.addEventListener('resize', function () { sizePhone(); drawChart(); });
    }
    sizePhone();
  }

  // ---------- 主循环 ----------
  var lastRenderT = 0, lastChartT = 0;
  function tick(now) {
    requestAnimationFrame(tick);
    if (!S.lastTick) S.lastTick = now;
    S.lastTick = now;
    if (S.demo) {
      var t = (now - S.demoStart) / 1000;
      ingestSim(wrap360(150 + t * 26), clamp(90 + 65 * Math.sin(t * 0.6), 0, 180), clamp(55 * Math.sin(t * 0.9), -90, 90));
      el.simA.value = wrap360(150 + t * 26); el.simB.value = (90 + 65 * Math.sin(t * 0.6)).toFixed(1); el.simG.value = (55 * Math.sin(t * 0.9)).toFixed(1);
    } else {
      stepSmooth();
    }
    if (S.dirty && now - lastRenderT > 16) {
      S.dirty = false; lastRenderT = now;
      render();
    }
    if (now - lastChartT > 400) { drawChart(); lastChartT = now; }
  }

  // ---------- 启动 ----------
  function boot() {
    bindCam(); bindSim(); bindUI(); bindResize();
    el.simA.value = 0; el.simB.value = 90; el.simG.value = 0;
    ingestSim(0, 90, 0);
    syncChip(); paintSeg(); camReset();
    updateSceneMsg();

    var wantSensor = SENSOR_SUPPORTED && MOBILE;
    if (wantSensor && NEED_GESTURE) {
      S.mode = 'sensor'; paintSeg(); setSensorState('wait');
    } else if (wantSensor) {
      setMode('sensor');
    } else if (!SENSOR_SUPPORTED) {
      setMode('sim'); showUnsupported();
    } else {
      setMode('sim');
      el.chipMode.textContent = '模拟模式 · 桌面无陀螺仪';
    }
    hzLoop();
    requestAnimationFrame(tick);
    drawChart();
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot);
  else boot();
})();
