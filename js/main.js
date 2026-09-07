/* main.js v4 — Canvas 2D 投影渲染
 * 数据: 事件(绝对优先单源) → 平滑 → 显示角(相对锚定基准)
 * 模型: 设备圆角棱柱顶点 → 设备轴矩阵(colX/Y/Z) → 轨道视角旋转 → 透视投影 → canvas 绘制
 * 每帧只画一张 canvas: 无 DOM 3D 图层、无逐帧 opacity 重绘。
 */
(function () {
  'use strict';
  var O = window.__gyroOrient || {};
  var deviceFrame = O.deviceFrameFromAngles;
  if (!deviceFrame) { console.error('orient.js 未加载'); return; }

  function $(id) { return document.getElementById(id); }
  function clamp(v, a, b) { return v < a ? a : (v > b ? b : v); }
  function wrap360(x) { x = x % 360; return x < 0 ? x + 360 : x; }
  function fmt(v, d) { return v.toFixed(d == null ? 1 : d); }

  var MOBILE = (typeof matchMedia === 'function' && matchMedia('(pointer:coarse)').matches) ||
               /iPhone|iPad|iPod|Android|Mobile/i.test(navigator.userAgent || '');
  var NEED_GESTURE = 'DeviceOrientationEvent' in window &&
                     typeof window.DeviceOrientationEvent.requestPermission === 'function';
  var SENSOR_SUPPORTED = 'DeviceOrientationEvent' in window;

  var el = {
    dotLive: $('dotLive'), chipMode: $('chipMode'), chipHz: $('chipHz'), chipAbs: $('chipAbs'),
    scene: $('scene'), model: $('model'), sceneMsg: $('sceneMsg'),
    vA: $('vA'), vB: $('vB'), vG: $('vG'),
    bvA: $('bvA'), bvB: $('bvB'), bvG: $('bvG'),
    fillA: $('fillA'), fillB: $('fillB'), fillG: $('fillG'),
    poseLine: $('poseLine'),
    msgSensor: $('msgSensor'), msgSensorText: $('msgSensorText'),
    msgError: $('msgError'), msgErrorText: $('msgErrorText'), btnEnable: $('btnEnable'),
    simA: $('simA'), simB: $('simB'), simG: $('simG'),
    simAv: $('simAv'), simBv: $('simBv'), simGv: $('simGv'),
    btnDemo: $('btnDemo'), btnZero: $('btnZero'),
    chart: $('chart'), chartPause: $('chartPause'), segMode: $('segMode'), helpOv: $('helpOv')
  };

  // ---------- 状态 ----------
  var S = {
    mode: 'sim', sensorState: 'idle',
    useAbsolute: null,
    uw: 0, db: 90, dg: 0, dispValid: false,
    tA: 0, tB: 90, tG: 0,
    solvePrev: { a: 0, b: 90, g: 0 },
    anchor: null, lastRaw: null,
    demo: false, demoStart: 0,
    chartPaused: false,
    chart: { t: [], a: [], b: [], g: [] },
    lastSample: 0, lastTick: 0, lastUI: 0, lastChart: 0,
    poseDirty: true, camDirty: true, sizeDirty: true,
    evCount: 0, evWinStart: performance.now(), lastEvT: 0
  };
  var CAM = { pitch: -14, yaw: 16, zoom: 40 };
  var TARGET = deviceFrame(0, 90, 0).css;

  // 模型几何(设备系 px, 顶=+y)
  var GEO = { hw: 48, hh: 103, t: 6.5, r: 12, outline: null };
  function buildOutline() {
    var hw = GEO.hw, hh = GEO.hh, r = Math.min(GEO.r, hw - 2, hh - 2);
    var pts = [];
    function arc(cx, cy, a0, a1) {
      for (var k = 0; k < 3; k++) {
        var a = a0 + (a1 - a0) * k / 3;
        pts.push({ x: cx + r * Math.cos(a), y: cy + r * Math.sin(a) });
      }
    }
    // 逆时针绕行(顶边向右、右下角、右边向下…), 角点坐标: 右上(hw,hh) 右下(hw,-hh) 左下(-hw,-hh) 左上(-hw,hh)
    // 顶边(左→右, 跳过左上到右上的弧在最后补) —— 用标准分段:
    // 以右上角为起点的参数化简洁做法: 四段直边 + 四段弧(每弧3点含终点)
    pts.push({ x: hw - r, y: hh });              // 顶边起点(从左上角开始走顶边)
    arc(hw - r, hh - r, -Math.PI / 2, 0);         // 右上角(补起始片段, 稍显重复可接受)
    // 简化: 重新生成干净顺序
    pts = [];
    function addLine(x1, y1, x2, y2, n) { for (var i = 0; i < n; i++) pts.push({ x: x1 + (x2 - x1) * i / n, y: y1 + (y2 - y1) * i / n }); }
    function addArc(cx, cy, a0, a1, n) { for (var i = 0; i < n; i++) { var a = a0 + (a1 - a0) * i / n; pts.push({ x: cx + r * Math.cos(a), y: cy + r * Math.sin(a) }); } }
    addLine(hw - r, hh, -hw + r, hh, 1);          // 顶边 左→右
    addArc(hw - r, hh - r, 0, Math.PI / 2, 3);     // 右上角 到 右边
    addLine(hw, hh - r, hw, -hh + r, 1);           // 右边 上→下
    addArc(hw - r, -hh + r, Math.PI / 2, Math.PI, 3); // 右下角
    addLine(hw - r, -hh, -hw + r, -hh, 1);         // 底边 右→左
    addArc(-hw + r, -hh + r, Math.PI, Math.PI * 1.5, 3); // 左下角
    addLine(-hw, -hh + r, -hw, hh - r, 1);         // 左边 下→上
    addArc(-hw + r, hh - r, Math.PI * 1.5, Math.PI * 2, 3); // 左上角(终点≈起点)
    GEO.outline = pts;
  }

  // ---------- 解算 css 列矩阵 → 显示欧拉角 ----------
  function flatCols(c) { return [c.x[0], c.x[1], c.x[2], c.y[0], c.y[1], c.y[2], c.z[0], c.z[1], c.z[2]]; }
  function solve3(A, bv) {
    var M = [A[0][0], A[0][1], A[0][2], A[1][0], A[1][1], A[1][2], A[2][0], A[2][1], A[2][2]], v = [bv[0], bv[1], bv[2]];
    for (var i = 0; i < 3; i++) {
      var piv = i;
      for (var k = i + 1; k < 3; k++) if (Math.abs(M[k * 3 + i]) > Math.abs(M[piv * 3 + i])) piv = k;
      if (piv !== i) {
        for (var k2 = 0; k2 < 3; k2++) { var tmp = M[i * 3 + k2]; M[i * 3 + k2] = M[piv * 3 + k2]; M[piv * 3 + k2] = tmp; }
        var tv = v[i]; v[i] = v[piv]; v[piv] = tv;
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
  function eulerFromCols(targetCols, guess) {
    var tgt = flatCols(targetCols);
    var a = guess.a, b = guess.b, g = guess.g;
    var eps = 1e-4, step = 0.7;
    for (var it = 0; it < 24; it++) {
      var cur = flatCols(deviceFrame(a, b, g).css);
      var r = new Array(9);
      for (var k = 0; k < 9; k++) r[k] = tgt[k] - cur[k];
      var Ja = flatCols(deviceFrame(a + eps, b, g).css);
      var Jb = flatCols(deviceFrame(a, b + eps, g).css);
      var Jg = flatCols(deviceFrame(a, b, g + eps).css);
      var JtJ = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], Jtr = [0, 0, 0];
      for (var i = 0; i < 9; i++) {
        var col = [(Ja[i] - cur[i]) / eps, (Jb[i] - cur[i]) / eps, (Jg[i] - cur[i]) / eps];
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

  // ---------- 传感器(单一数据源, 绝对优先) ----------
  function useThisEvent(e) {
    if (e.absolute) { S.useAbsolute = true; return true; }
    if (S.useAbsolute === true) return false;
    if (S.useAbsolute === null) S.useAbsolute = false;
    return true;
  }
  function onOrient(e) {
    var a = e.alpha, b = e.beta, g = e.gamma;
    if (a == null || b == null || g == null || !isFinite(a + b + g)) return;
    if (!useThisEvent(e)) return;
    S.evCount++; S.lastEvT = performance.now();
    S.lastRaw = { a: wrap360(a), b: clamp(b, -180, 180), g: clamp(g, -180, 180) };
    handleRaw(S.lastRaw.a, S.lastRaw.b, S.lastRaw.g);
  }
  function attachSensor() {
    window.addEventListener('deviceorientation', onOrient, true);
    if ('ondeviceorientationabsolute' in window) window.addEventListener('deviceorientationabsolute', onOrient, true);
  }
  function detachSensor() {
    window.removeEventListener('deviceorientation', onOrient, true);
    window.removeEventListener('deviceorientationabsolute', onOrient, true);
  }

  // ---------- 原始角 → 目标显示角 ----------
  var anchorBuf = [], anchorT0 = 0;
  function handleRaw(a, b, g) {
    if (!S.anchor) { collectAnchor(a, b, g); return; }
    var cols = deviceFrame(a, b, g).css;
    var D = O.mat3ApplyToCols(S.anchor, cols);
    var sol = eulerFromCols(D, S.solvePrev);
    S.solvePrev = sol;
    S.tA = sol.a; S.tB = sol.b; S.tG = sol.g;
    if (!S.dispValid) { S.uw = sol.a; S.db = sol.b; S.dg = sol.g; S.dispValid = true; S.poseDirty = true; }
  }
  function collectAnchor(a, b, g) {
    var now = performance.now();
    if (!anchorT0) anchorT0 = now;
    anchorBuf.push({ t: now, a: a, b: b, g: g });
    while (anchorBuf.length && now - anchorBuf[0].t > 1600) anchorBuf.shift();
    if (now - anchorT0 < 350) return;
    var best = null, bestScore = Infinity;
    for (var i = 1; i < anchorBuf.length - 1; i++) {
      var p = anchorBuf[i], score = 0;
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
    var bcols = deviceFrame(best.a, best.b, best.g).css;
    S.anchor = O.mat3Mul3(O.mat3FromCols(TARGET), O.mat3Transpose(O.mat3FromCols(bcols)));
    S.solvePrev = { a: 0, b: 90, g: 0 };
    anchorBuf = [];
    syncAbsLabel(); updateSceneMsg();
    if (S.sensorState === 'running') setSensorState('running');
  }
  function reAnchorNow() {
    if (S.lastRaw) {
      var cols = deviceFrame(S.lastRaw.a, S.lastRaw.b, S.lastRaw.g).css;
      S.anchor = O.mat3Mul3(O.mat3FromCols(TARGET), O.mat3Transpose(O.mat3FromCols(cols)));
    } else S.anchor = O.identityMat3();
    S.solvePrev = { a: 0, b: 90, g: 0 };
    syncAbsLabel(); updateSceneMsg();
  }

  // ---------- 平滑(每 rAF 一步, 指数) ----------
  function stepSmooth(dt) {
    if (S.mode !== 'sensor' || !S.dispValid) { S.tA = S.uw; S.tB = S.db; S.tG = S.dg; return; }
    var kA = 1 - Math.exp(-dt / 0.10);   // α 慢一点(磁罗盘噪)
    var kB = 1 - Math.exp(-dt / 0.055);  // β/γ 快(跟手)
    var cur = wrap360(S.uw);
    var diff = ((S.tA - cur + 540) % 360) - 180;
    var dUw = diff * kA;
    var nB = S.db + (S.tB - S.db) * kB;
    var nG = S.dg + (S.tG - S.dg) * kB;
    if (Math.abs(dUw) > 0.002 || Math.abs(nB - S.db) > 0.002 || Math.abs(nG - S.dg) > 0.002) S.poseDirty = true;
    S.uw += dUw; S.db = nB; S.dg = nG;
  }
  function display() { return { a: wrap360(S.uw), b: clamp(S.db, -180, 180), g: clamp(S.dg, -180, 180) }; }
  function ingestSim(a, b, g) {
    S.uw = wrap360(a); S.db = clamp(b, -180, 180); S.dg = clamp(g, -180, 180);
    S.dispValid = true; S.poseDirty = true;
  }

  // ---------- 模式 / 授权 ----------
  function syncAbsLabel() {
    el.chipAbs.textContent = S.mode !== 'sensor' ? '—' : (S.anchor ? '起始基准' : '对准中…');
  }
  function syncChip() {
    syncAbsLabel();
    if (S.mode === 'sensor' && S.sensorState === 'running') el.dotLive.className = 'dot on';
  }
  function updateSceneMsg() {
    var m = el.sceneMsg;
    if (!m) return;
    if (S.mode !== 'sensor') { m.textContent = '模拟模式：右侧滑杆 / 演示摇摆驱动模型'; return; }
    if (S.sensorState === 'denied' || S.sensorState === 'unsupported' || S.sensorState === 'lost') m.textContent = '传感器暂不可用，可先切“模拟”';
    else if (S.sensorState === 'wait') m.textContent = '点击右上角「传感器」并在弹窗允许后开始';
    else if (!S.anchor) m.textContent = '正在对准… 请像平时看手机那样竖握并拿稳（约 1 秒）';
    else m.textContent = '基准 = 当前拿姿 · 之后转动随之变化 · 双击画面可重新对准';
  }
  function paintSeg() {
    document.querySelectorAll('#segMode button').forEach(function (b) { b.classList.toggle('active', b.dataset.mode === S.mode); });
  }
  function setSensorState(st) {
    S.sensorState = st;
    if (st === 'running') el.dotLive.className = 'dot on';
    else if (st === 'lost') el.dotLive.className = 'dot warn';
    else el.dotLive.className = 'dot';
    if (S.mode !== 'sensor') return;
    el.msgError.classList.remove('show');
    el.msgSensor.classList.remove('show');
    el.btnEnable.style.display = 'none';
    switch (st) {
      case 'running': el.chipMode.textContent = S.anchor ? '传感器·运行中' : '传感器·对准中…'; break;
      case 'wait':
        el.chipMode.textContent = '传感器·待授权';
        el.msgSensorText.textContent = 'iOS 需要授权：点击「传感器」后弹窗选“允许”，然后竖握拿稳约 1 秒。';
        el.msgSensor.classList.add('show');
        break;
      case 'denied':
        el.chipMode.textContent = '传感器·已拒绝';
        el.msgErrorText.textContent = '权限被拒：页面需 HTTPS/localhost；iOS 需在 Safari 中允许“运动与健身”。';
        el.msgError.classList.add('show');
        break;
      case 'lost':
        el.dotLive.className = 'dot warn';
        el.chipMode.textContent = '传感器·无数据';
        el.btnEnable.style.display = '';
        el.msgErrorText.textContent = '传感器一直没有数据。可点击重试，或切换模拟模式。';
        el.msgError.classList.add('show');
        break;
      default: el.chipMode.textContent = '传感器'; break;
    }
    updateSceneMsg();
  }
  function showUnsupported() {
    el.chipMode.textContent = '模拟模式 · 传感器不可用';
    el.msgSensor.classList.remove('show');
    el.msgError.classList.remove('show');
    el.msgErrorText.textContent = '当前浏览器不支持 DeviceOrientation，或不在安全上下文（需 HTTPS/localhost）。已自动使用模拟。';
    el.msgError.classList.add('show');
    updateSceneMsg();
  }
  function resetSensorSession() {
    S.anchor = null; S.useAbsolute = null; S.dispValid = false;
    S.solvePrev = { a: 0, b: 90, g: 0 };
    S.tA = 0; S.tB = 90; S.tG = 0;
    anchorBuf = []; anchorT0 = 0; S.lastRaw = null;
    S.uw = 0; S.db = 90; S.dg = 0;
    S.poseDirty = true;
  }
  function setMode(m, fromGesture) {
    if (m === S.mode) { if (m === 'sensor') tryEnableSensor(fromGesture); return; }
    S.mode = m;
    paintSeg();
    if (m === 'sensor') { resetSensorSession(); tryEnableSensor(fromGesture); }
    else {
      detachSensor(); resetSensorSession();
      el.dotLive.className = 'dot';
      el.msgSensor.classList.remove('show'); el.msgError.classList.remove('show');
      el.btnEnable.style.display = 'none';
      el.chipMode.textContent = '模拟模式';
      syncChip(); updateSceneMsg();
    }
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

  // ---------- Canvas 模型渲染 ----------
  var ctx = null, W = 0, H = 0, cx = 0, cy = 0, P = 1250;
  var R = null, fwd = [0, 0, 1];   // 视角旋转矩阵(行主序) 与 前向向量
  var C = { x: null, y: null, z: null }; // 设备列(当前显示角)
  function devPt(x, y, z) {
    var w = [C.x[0] * x + C.y[0] * y + C.z[0] * z,
             C.x[1] * x + C.y[1] * y + C.z[1] * z,
             C.x[2] * x + C.y[2] * y + C.z[2] * z];
    var u = [
      R[0] * w[0] + R[1] * w[1] + R[2] * w[2],
      R[3] * w[0] + R[4] * w[1] + R[5] * w[2],
      R[6] * w[0] + R[7] * w[1] + R[8] * w[2]
    ];
    u[2] += CAM.zoom;
    return u;
  }
  function applyCamMat() {
    var Rx = O.rotMat3Axis([1, 0, 0], CAM.pitch);
    var Ry = O.rotMat3Axis([0, 1, 0], CAM.yaw);
    R = O.mat3Mul3(Rx, Ry);
    fwd = [R[2], R[5], R[8]]; // R·ez(第三列)
  }
  function setupCtx() {
    if (!ctx) ctx = el.model.getContext('2d');
    var dpr = window.devicePixelRatio || 1;
    var w = el.scene.clientWidth, h = el.scene.clientHeight;
    if (W !== w || H !== h) {
      W = w; H = h;
      el.model.width = Math.round(w * dpr);
      el.model.height = Math.round(h * dpr);
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      cx = w / 2; cy = h / 2;
      var pw = clamp(Math.min(w * 0.30, h * 0.42), 54, 180);
      GEO.hw = pw / 2; GEO.hh = pw * 2.15 / 2; GEO.t = pw * 0.13;
      GEO.r = Math.min(pw * 0.16, GEO.hw - 2, GEO.hh - 2);
      buildOutline();
      S.sizeDirty = true;
    }
  }
  function proj(u) {
    var z = u[2];
    if (z > P - 60) return null;
    var sc = P / (P - z);
    return { x: cx + u[0] * sc, y: cy + u[1] * sc, s: sc };
  }
  function drawModel() {
    setupCtx();
    if (!W) return;
    applyCamMat();
    ctx.clearRect(0, 0, W, H);

    var o = GEO.outline, t2 = GEO.t / 2, n = o.length;
    // 投影: 前(后)环的点(设备 x,y → css 世界 → 视角)
    var ptsF = new Array(n), ptsB = new Array(n);
    for (var i = 0; i < n; i++) { ptsF[i] = proj(devPt(o[i].x, o[i].y, t2)); ptsB[i] = proj(devPt(o[i].x, o[i].y, -t2)); }
    function avgZ(pts) { var s = 0, c2 = 0; for (var i = 0; i < pts.length; i++) if (pts[i]) { s += pts[i]._z; c2++; } return c2 ? s / c2 : -1e9; }
    // 记录深度
    for (var i = 0; i < n; i++) { if (ptsF[i]) ptsF[i]._z = devPt(o[i].x, o[i].y, t2)[2]; if (ptsB[i]) ptsB[i]._z = devPt(o[i].x, o[i].y, -t2)[2]; }

    var faces = [];
    faces.push({ type: 'back', pts: ptsB });
    for (var i = 0; i < n; i++) {
      var j = (i + 1) % n;
      var ok1 = ptsF[i] && ptsF[j] && ptsB[i] && ptsB[j];
      if (!ok1) continue;
      faces.push({ type: 'side', i: i, j: j });
    }
    faces.push({ type: 'front', pts: ptsF });
    faces.sort(function (fa, fb) {
      var za = fa.type === 'side' ? avgSideZ(fa) : avgZ(fa.pts);
      var zb = fb.type === 'side' ? avgSideZ(fb) : avgZ(fb.pts);
      return zb - za;
    });
    function avgSideZ(f) { var a = devPt(o[f.i].x, o[f.i].y, t2)[2], b = devPt(o[f.j].x, o[f.j].y, t2)[2], cc = devPt(o[f.i].x, o[f.i].y, -t2)[2]; return (a + b + cc) / 3; }
    for (var f = 0; f < faces.length; f++) drawFace(faces[f], o);
    drawRays();
    drawDeco();
  }
  function pathPoly(pts) {
    ctx.beginPath();
    var started = false;
    for (var i = 0; i < pts.length; i++) {
      var p = pts[i];
      if (!p) continue;
      if (!started) { ctx.moveTo(p.x, p.y); started = true; }
      else ctx.lineTo(p.x, p.y);
    }
  }
  function drawFace(face, o) {
    if (face.type === 'front' || face.type === 'back') {
      var pts = face.pts;
      var any = false;
      for (var i = 0; i < pts.length; i++) if (pts[i]) any = true;
      if (!any) return;
      pathPoly(pts);
      if (face.type === 'front') {
        // 屏幕渐变(沿设备上方向)
        var ta = devPt(0, GEO.hh, GEO.t / 2), ba = devPt(0, -GEO.hh, GEO.t / 2);
        var tp = proj(ta), bp = proj(ba);
        var grd = ctx.createLinearGradient(tp ? tp.x : cx, tp ? tp.y : cy, bp ? bp.x : cx, bp ? bp.y : cy);
        grd.addColorStop(0, '#2b3856');
        grd.addColorStop(0.5, '#1a2236');
        grd.addColorStop(1, '#312a56');
        ctx.fillStyle = grd;
      } else {
        var grdB = ctx.createLinearGradient(0, 0, 0, H * 0.5);
        grdB.addColorStop(0, '#dfe6f2');
        grdB.addColorStop(1, '#9aa8c0');
        ctx.fillStyle = grdB;
      }
      ctx.closePath();
      ctx.fill();
      ctx.lineWidth = 1;
      ctx.strokeStyle = 'rgba(0,0,0,.35)';
      ctx.stroke();
    } else {
      // 侧边
      var oi = o[face.i], oj = o[face.j];
      var pts = [proj(devPt(oi.x, oi.y, GEO.t / 2)), proj(devPt(oj.x, oj.y, GEO.t / 2)), proj(devPt(oj.x, oj.y, -GEO.t / 2)), proj(devPt(oi.x, oi.y, -GEO.t / 2))];
      for (var k = 0; k < 4; k++) if (!pts[k]) return;
      ctx.beginPath();
      ctx.moveTo(pts[0].x, pts[0].y);
      ctx.lineTo(pts[1].x, pts[1].y);
      ctx.lineTo(pts[2].x, pts[2].y);
      ctx.lineTo(pts[3].x, pts[3].y);
      ctx.closePath();
      // 侧边按位置固定深浅: 顶亮/右中/其余暗
      var mx = (oi.x + oj.x) / 2, my = (oi.y + oj.y) / 2;
      var isTop = Math.abs(my) > Math.abs(mx) && my > 0;
      var isRight = Math.abs(mx) > Math.abs(my) && mx > 0;
      ctx.fillStyle = isTop ? '#5d6a83' : (isRight ? '#3c4558' : '#232a38');
      ctx.fill();
      ctx.strokeStyle = 'rgba(0,0,0,.25)';
      ctx.lineWidth = 0.75;
      ctx.stroke();
    }
  }
  function drawRays() {
    var rays = [
      { a: C.x, from: [GEO.hw * 0.92, 0, 0], to: [GEO.hw + 22, 0, 0], col: '#ff5b5b', lit: '#ffb3b3' },
      { a: C.y, from: [0, GEO.hh * 0.92, 0], to: [0, GEO.hh + 22, 0], col: '#37e787', lit: '#b6ffd6' },
      { a: C.z, from: [0, 0, GEO.t * 0.4 + 2], to: [0, 0, 34], col: '#39c5ff', lit: '#b3e6ff' }
    ];
    for (var i = 0; i < rays.length; i++) {
      var rr = rays[i];
      var dirW = [rr.a[0], rr.a[1], rr.a[2]];
      // 指向观察者才画(避免穿过机身)
      var dotv = dirW[0] * fwd[0] + dirW[1] * fwd[1] + dirW[2] * fwd[2];
      if (dotv < 0.06) continue;
      var p1 = proj(devPt(rr.from[0], rr.from[1], rr.from[2]));
      var p2 = proj(devPt(rr.to[0], rr.to[1], rr.to[2]));
      if (!p1 || !p2) continue;
      ctx.save();
      ctx.lineCap = 'round';
      ctx.strokeStyle = rr.col;
      ctx.globalAlpha = 0.35;
      ctx.lineWidth = 6;
      ctx.beginPath(); ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y); ctx.stroke();
      ctx.globalAlpha = 1;
      ctx.lineWidth = 2.2;
      ctx.stroke();
      ctx.globalAlpha = 1;
      ctx.fillStyle = rr.lit;
      ctx.beginPath(); ctx.arc(p2.x, p2.y, 2.6, 0, Math.PI * 2); ctx.fill();
      ctx.restore();
    }
  }
  function drawDeco() {
    // 正面屏幕细节(开槽/摄像头/Home 条)与背面标: 取 front 中心深度缩放
    var fc = proj(devPt(0, 0, GEO.t / 2));
    if (!fc) return;
    var s = fc.s;
    var upA = proj(devPt(0, 30, GEO.t / 2)), cA = proj(devPt(0, 0, GEO.t / 2));
    if (!upA || !cA) return;
    var ang = Math.atan2(upA.y - cA.y, upA.x - cA.x);
    var isFrontFacing = dotVec(C.z, fwd) > 0.1;
    ctx.save();
    ctx.translate(fc.x, fc.y);
    ctx.rotate(ang);
    if (isFrontFacing) {
      // 开槽+摄像头(设备顶部)
      ctx.fillStyle = 'rgba(2,4,8,.88)';
      roundRect(-GEO.hw * 0.26 * s, -GEO.hh * 0.88 * s, GEO.hw * 0.52 * s, 5.5 * s, 3);
      ctx.fill();
      ctx.beginPath();
      ctx.arc(GEO.hw * 0.24 * s, -GEO.hh * 0.88 * s, 2.4 * s, 0, Math.PI * 2);
      ctx.fillStyle = '#0a0f1a';
      ctx.fill();
      ctx.beginPath();
      ctx.arc(-GEO.hw * 0.28 * s, -GEO.hh * 0.88 * s, 2.2 * s, 0, Math.PI * 2);
      ctx.fillStyle = '#33415f';
      ctx.fill();
      // Home 条(设备底部)
      roundRect(-GEO.hw * 0.3 * s, GEO.hh * 0.84 * s, GEO.hw * 0.6 * s, 4 * s, 2);
      ctx.fillStyle = 'rgba(255,255,255,.22)';
      ctx.fill();
      // 屏幕高光
      ctx.globalAlpha = 0.12;
      ctx.beginPath();
      ctx.moveTo(-GEO.hw * 0.5 * s, -GEO.hh * 0.4 * s);
      ctx.lineTo(GEO.hw * 0.55 * s, -GEO.hh * 0.5 * s);
      ctx.lineTo(GEO.hw * 0.3 * s, GEO.hh * 0.3 * s);
      ctx.lineTo(-GEO.hw * 0.45 * s, GEO.hh * 0.35 * s);
      ctx.closePath();
      ctx.fillStyle = '#ffffff';
      ctx.fill();
      ctx.globalAlpha = 1;
    } else {
      // 背面: 摄像方块 + 字(设备顶部方向)
      roundRect(-GEO.hw * 0.3 * s, -GEO.hh * 0.8 * s, GEO.hw * 0.6 * s, GEO.hw * 0.5 * s, 4 * s);
      ctx.fillStyle = 'rgba(40,52,80,.35)';
      ctx.fill();
      ctx.beginPath();
      ctx.arc(-GEO.hw * 0.12 * s, -GEO.hh * 0.8 * s + GEO.hw * 0.25 * s, 5.5 * s, 0, Math.PI * 2);
      ctx.fillStyle = '#111722';
      ctx.fill();
      ctx.beginPath();
      ctx.arc(GEO.hw * 0.14 * s, -GEO.hh * 0.8 * s + GEO.hw * 0.25 * s, 3.4 * s, 0, Math.PI * 2);
      ctx.fillStyle = '#0b0f18';
      ctx.fill();
    }
    ctx.restore();
  }
  function roundRect(x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r);
    ctx.arcTo(x, y, x + w, y, r);
    ctx.closePath();
  }
  function dotVec(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

  // ---------- UI 数字/仪表(≤~33ms) ----------
  function updateUI() {
    var ang = display();
    el.vA.textContent = fmt(ang.a); el.vB.textContent = fmt(ang.b); el.vG.textContent = fmt(ang.g);
    el.bvA.textContent = fmt(ang.a) + '°'; el.bvB.textContent = fmt(ang.b) + '°'; el.bvG.textContent = fmt(ang.g) + '°';
    el.simAv.textContent = fmt(parseFloat(el.simA.value)) + '°';
    el.simBv.textContent = fmt(parseFloat(el.simB.value)) + '°';
    el.simGv.textContent = fmt(parseFloat(el.simG.value)) + '°';
    el.fillA.style.width = clamp(ang.a / 360 * 100, 0, 100) + '%';
    el.fillB.style.width = clamp(ang.b / 180 * 100, 0, 100) + '%';
    var gp = ang.g / 180 * 100;
    if (ang.g >= 0) { el.fillG.style.left = '50%'; el.fillG.style.width = clamp(gp, 0, 100) + '%'; }
    else { el.fillG.style.left = (50 + gp) + '%'; el.fillG.style.width = clamp(-gp, 0, 100) + '%'; }
    var b = ang.b, g2 = ang.g, tags = [];
    if (b < 20) tags.push(['接近平放', 'var(--a)']);
    else if (b > 160) tags.push(['接近翻转', 'var(--g)']);
    else if (b >= 70 && b <= 110) tags.push(['竖立', 'var(--ok)']);
    else tags.push(['倾斜', 'var(--b)']);
    if (g2 > 12) tags.push(['右倾', 'var(--g)']);
    else if (g2 < -12) tags.push(['左倾', 'var(--g)']);
    else tags.push(['左右端正', 'var(--b)']);
    el.poseLine.innerHTML = '';
    tags.forEach(function (tg) {
      var sp = document.createElement('span');
      sp.className = 'tag';
      sp.style.borderColor = tg[1]; sp.style.color = tg[1];
      sp.textContent = tg[0];
      el.poseLine.appendChild(sp);
    });
  }

  // ---------- 曲线 ----------
  function sampleChart() {
    var now = performance.now();
    if (S.chartPaused || now - S.lastSample < 33) return;
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
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
    var g = cv.getContext('2d');
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, w, h);
    var c = S.chart;
    if (!c.t.length) {
      g.fillStyle = 'rgba(255,255,255,.14)'; g.font = '11px sans-serif';
      g.textAlign = 'center'; g.fillText('等待数据…', w / 2, h / 2); g.textAlign = 'left';
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
      g.strokeStyle = 'rgba(255,255,255,.07)';
      g.beginPath();
      g.moveTo(padL, y1); g.lineTo(w - padR, y1);
      g.moveTo(padL, y0 + lane / 2); g.lineTo(w - padR, y0 + lane / 2);
      g.stroke();
      var ser = series[s];
      g.strokeStyle = ser.col; g.lineWidth = 1.6;
      g.beginPath();
      var started = false;
      var step = Math.max(1, Math.floor(c.t.length / (w * 2)));
      for (var i = 0; i < c.t.length; i += step) {
        var v = ser.d[i];
        var px = xAt(c.t[i]);
        var py = y1 - 5 - ((v - ser.min) / (ser.max - ser.min)) * (lane - 10);
        if (!started) { g.moveTo(px, py); started = true; }
        else g.lineTo(px, py);
      }
      g.stroke();
      var last = ser.d[ser.d.length - 1];
      g.fillStyle = ser.col;
      g.beginPath(); g.arc(xAt(t1), y1 - 5 - ((last - ser.min) / (ser.max - ser.min)) * (lane - 10), 2, 0, Math.PI * 2); g.fill();
    }
  }

  // ---------- 视角控制 ----------
  function bindCam() {
    var drag = null, taps = 0, tapT = 0;
    el.scene.addEventListener('pointerdown', function (e) {
      drag = { x: e.clientX, y: e.clientY, moved: false };
      try { el.scene.setPointerCapture(e.pointerId); } catch (err) {}
      el.scene.classList.add('dragging');
    });
    el.scene.addEventListener('pointermove', function (e) {
      if (!drag) return;
      var dx = e.clientX - drag.x, dy = e.clientY - drag.y;
      if (Math.abs(dx) + Math.abs(dy) > 3) drag.moved = true;
      CAM.yaw += dx * 0.32;
      CAM.pitch = clamp(CAM.pitch - dy * 0.32, -88, 45);
      drag.x = e.clientX; drag.y = e.clientY;
      S.camDirty = true;
    });
    el.scene.addEventListener('pointerup', function (e) {
      var wasTap = drag && !drag.moved;
      drag = null;
      el.scene.classList.remove('dragging');
      try { el.scene.releasePointerCapture(e.pointerId); } catch (err) {}
      if (wasTap) {
        var now = performance.now();
        if (now - tapT < 350 && ++taps >= 2) {
          taps = 0;
          if (S.mode === 'sensor' && S.anchor) reAnchorNow();
          else camReset();
        } else { if (now - tapT >= 350) taps = 1; tapT = now; }
      }
    });
    el.scene.addEventListener('pointercancel', function () { drag = null; el.scene.classList.remove('dragging'); });
    el.scene.addEventListener('wheel', function (e) {
      e.preventDefault();
      CAM.zoom = clamp(CAM.zoom + e.deltaY * 0.6, -850, 700);
      S.camDirty = true;
    }, { passive: false });
  }
  function camReset() { CAM.pitch = -14; CAM.yaw = 16; CAM.zoom = 40; S.camDirty = true; }

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

  // ---------- UI / 尺寸 ----------
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
    if ('ResizeObserver' in window) {
      var ro = new ResizeObserver(function () { S.sizeDirty = true; });
      ro.observe(el.scene);
    } else {
      window.addEventListener('resize', function () { S.sizeDirty = true; });
    }
  }

  // ---------- 主循环 ----------
  function tick(now) {
    requestAnimationFrame(tick);
    if (!S.lastTick) S.lastTick = now;
    var dt = clamp((now - S.lastTick) / 1000, 0.001, 0.1);
    S.lastTick = now;
    if (S.demo) {
      var t = (now - S.demoStart) / 1000;
      ingestSim(wrap360(150 + t * 26), clamp(90 + 65 * Math.sin(t * 0.6), 0, 180), clamp(55 * Math.sin(t * 0.9), -90, 90));
      el.simA.value = S.uw; el.simB.value = S.db.toFixed(1); el.simG.value = S.dg.toFixed(1);
    } else {
      stepSmooth(dt);
    }
    var changed = S.poseDirty || S.camDirty || S.sizeDirty;
    if (changed) {
      var ang = display();
      var f = deviceFrame(ang.a, ang.b, ang.g);
      C.x = f.css.x; C.y = f.css.y; C.z = f.css.z;
      drawModel();
      S.poseDirty = false; S.camDirty = false; S.sizeDirty = false;
      if (now - S.lastUI > 33) { updateUI(); S.lastUI = now; }
      sampleChart();
    }
    if (now - S.lastChart > 400) { drawChart(); S.lastChart = now; }
  }
  // ---------- 启动 ----------
  function boot() {
    bindCam(); bindSim(); bindUI(); bindResize();
    el.simA.value = 0; el.simB.value = 90; el.simG.value = 0;
    ingestSim(0, 90, 0);
    syncChip(); paintSeg(); camReset(); updateSceneMsg();

    var wantSensor = SENSOR_SUPPORTED && MOBILE;
    if (wantSensor && NEED_GESTURE) { S.mode = 'sensor'; paintSeg(); setSensorState('wait'); }
    else if (wantSensor) setMode('sensor');
    else if (!SENSOR_SUPPORTED) { setMode('sim'); showUnsupported(); }
    else { setMode('sim'); el.chipMode.textContent = '模拟模式 · 桌面无陀螺仪'; }

    hzLoop();
    requestAnimationFrame(tick);
    drawChart();
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot);
  else boot();
})();
