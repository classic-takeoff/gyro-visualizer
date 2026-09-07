/* main.js — 陀螺仪姿态可视化主逻辑(经典脚本, 无模块依赖)
 * 渲染流程: 传感器/模拟 → raw 角(α,β,γ) → [传感器才做]指数平滑 → 模型角
 *           → deviceFrameFromAngles → 校准矩阵 X 修正列 → CSS matrix3d + 面光照
 *           校准按钮: 对准我/↺90/↻90/重置(把当前真机姿态映射到"竖立正对观察者") */
(function () {
  'use strict';

  var O = window.__gyroOrient || {};
  var deviceFrame = O.deviceFrameFromAngles;
  if (!deviceFrame) { console.error('orient.js 未加载'); return; }

  // ---------- 小工具 ----------
  function $(id) { return document.getElementById(id); }
  function clamp(v, a, b) { return v < a ? a : (v > b ? b : v); }
  function fmt(v, d) { d = d == null ? 1 : d; return v.toFixed(d); }
  function wrap360(x) { x = x % 360; return x < 0 ? x + 360 : x; }
  function norm3(v) { return O.norm(v); }
  function dot3(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

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
    compassSvg: $('compassSvg'), helpOv: $('helpOv'),
    btnCalib: $('btnCalib'), btnSpinL: $('btnSpinL'), btnSpinR: $('btnSpinR'),
    btnCamReset: $('btnCamReset'), fixChip: $('fixChip'),
    smoothR: $('smoothR'), smVal: $('smVal')
  };

  // ---------- 状态 ----------
  var S = {
    mode: 'sim',                 // 'sim' | 'sensor'
    sensorState: 'idle',
    absolute: null,
    src: 'sim',                  // 当前数据来源
    // 原始角(传感器/模拟直读)
    rawA: 0, rawB: 90, rawG: 0,
    // 平滑后的连续值
    uw: 0, dB: 90, dG: 0, smInit: false,
    smooth: 0.45,                // 0..1 平滑强度(滑杆 0..100 / 100)
    X: null,                     // 校准旋转矩阵(行主序 9), null=单位阵
    dirty: false, demo: false, demoStart: 0,
    chartPaused: false,
    chart: { t: [], a: [], b: [], g: [] },
    lastSample: 0, lastTick: 0,
    evCount: 0, evWinStart: performance.now(), lastEvT: 0
  };
  var CAM = { pitch: -14, yaw: 18, zoom: -20 };
  var TARGET_COLS = deviceFrame(0, 90, 0).css;   // “竖立正对观察者”基准姿态

  // ---------- 数据输入 ----------
  // src: 'sensor' | 'sim' | 'demo'
  function ingest(aDeg, bDeg, gDeg, src) {
    S.rawA = wrap360(aDeg); S.rawB = clamp(bDeg, -180, 180); S.rawG = clamp(gDeg, -90, 90);
    S.src = src;
    if (src === 'sensor') {
      if (!S.smInit) { S.uw = S.rawA; S.dB = S.rawB; S.dG = S.rawG; S.smInit = true; }
    } else {
      // 模拟/演示不过平滑, 直通
      S.uw = S.rawA; S.dB = S.rawB; S.dG = S.rawG; S.smInit = false;
      S.dirty = true;
    }
  }
  function stepSmooth(dt) {
    if (!S.smInit || S.src !== 'sensor') return;
    var s = S.smooth;
    if (s <= 0.001) {
      if (Math.abs(wrap360(S.uw) - S.rawA) > 0.01 || Math.abs(S.dB - S.rawB) > 0.01 || Math.abs(S.dG - S.rawG) > 0.01) S.dirty = true;
      S.uw = S.rawA; S.dB = S.rawB; S.dG = S.rawG; return;
    }
    var kA = 1 - Math.exp(-dt / (0.008 + s * 0.30));
    var kB = 1 - Math.exp(-dt / (0.005 + s * 0.16));
    var kG = kB;
    var cur = wrap360(S.uw);
    var diff = ((S.rawA - cur + 540) % 360) - 180;
    S.uw += diff * kA;
    var nB = S.dB + (S.rawB - S.dB) * kB;
    var nG = S.dG + (S.rawG - S.dG) * kG;
    if (Math.abs(nB - S.dB) > 0.004 || Math.abs(nG - S.dG) > 0.004 || Math.abs(diff * kA) > 0.004) S.dirty = true;
    S.dB = nB; S.dG = nG;
  }
  function displayAngles() {
    return { a: wrap360(S.uw), b: clamp(S.dB, -180, 180), g: clamp(S.dG, -90, 90) };
  }

  // ---------- 传感器 ----------
  function onOrient(e) {
    var a = e.alpha, b = e.beta, g = e.gamma;
    if (a == null || b == null || g == null) return;
    if (!isFinite(a + b + g)) return;
    S.absolute = !!e.absolute && e.absolute !== null;
    el.chipAbs.textContent = S.absolute ? '绝对' : '相对';
    S.evCount++; S.lastEvT = performance.now();
    ingest(a, b, g, 'sensor');
  }
  function attachSensor() {
    window.addEventListener('deviceorientation', onOrient, true);
    if ('ondeviceorientationabsolute' in window) window.addEventListener('deviceorientationabsolute', onOrient, true);
    return true;
  }
  function detachSensor() {
    window.removeEventListener('deviceorientation', onOrient, true);
    window.removeEventListener('deviceorientationabsolute', onOrient, true);
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
  function showUnsupported() {
    S.sensorState = 'unsupported';
    el.chipMode.textContent = '模拟模式 · 传感器不可用';
    el.msgSensor.classList.remove('show');
    el.msgError.classList.remove('show');
    el.msgErrorText.textContent = '当前浏览器不支持 DeviceOrientation，或页面不在安全上下文（需 HTTPS 或 localhost）。已自动使用模拟模式。';
    el.msgError.classList.add('show');
  }
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
      case 'running': el.chipMode.textContent = '传感器·运行中'; break;
      case 'wait':
        el.chipMode.textContent = '传感器·待授权';
        el.msgSensorText.textContent = 'iOS 需要授权：点击右上角「传感器」/下方按钮后在系统弹窗选“允许”。若模型方向不对，数据到手后点 🎯对准我。';
        el.msgSensor.classList.add('show');
        break;
      case 'denied':
        el.chipMode.textContent = '传感器·已拒绝';
        el.msgErrorText.textContent = '传感器权限被拒绝。请检查：页面需 HTTPS 或 localhost；iOS 需在 Safari 中打开并在系统设置允许“运动与健身”。可切回模拟模式。';
        el.msgError.classList.add('show');
        break;
      case 'unsupported':
        el.chipMode.textContent = '传感器·不支持';
        el.msgErrorText.textContent = '当前浏览器不支持 DeviceOrientation。';
        el.msgError.classList.add('show');
        break;
      case 'lost':
        el.dotLive.className = 'dot warn';
        el.chipMode.textContent = '传感器·无数据';
        el.btnEnable.style.display = '';
        el.msgErrorText.textContent = '传感器一直没有数据。可点击重试，或切换模拟模式。';
        el.msgError.classList.add('show');
        break;
      case 'idle': el.chipMode.textContent = '传感器·空闲'; break;
    }
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
      tryEnableSensor(fromGesture);
    } else {
      detachSensor();
      S.absolute = null; S.smInit = false; S.src = 'sim';
      el.dotLive.className = 'dot';
      el.msgSensor.classList.remove('show');
      el.msgError.classList.remove('show');
      el.btnEnable.style.display = 'none';
      el.chipMode.textContent = '模拟模式';
      S.sensorState = 'idle';
      syncChip();
    }
  }

  // ---------- 校准(修正矩阵 X) ----------
  function isCalibrated() { return !!S.X; }
  function syncFixChip() { el.fixChip.style.display = isCalibrated() ? 'inline' : 'none'; }
  function calibToCurrent() {
    // 目标: 让"当前真实姿态"显示为 TARGET(竖立正对); X = T · M0ᵀ
    var cur = displayAngles();
    var M0 = deviceFrame(cur.a, cur.b, cur.g).css;
    var X = O.mat3Mul3(O.mat3FromCols(TARGET_COLS), O.mat3Transpose(O.mat3FromCols(M0)));
    S.X = X;
    S.dirty = true;
    syncFixChip();
    // 校准后把相机转到正面视角方便观察
    CAM.yaw = 0; CAM.pitch = 0; CAM.zoom = -10;
    applyCam();
  }
  function spinModel(dir) {
    var cur = displayAngles();
    var F = correctedCols(deviceFrame(cur.a, cur.b, cur.g).css);
    var axis = norm3(F.z);
    var deg = (dir === 'L') ? -90 : 90;
    var R = O.rotMat3Axis(axis, deg);
    var curX = S.X || O.identityMat3();
    S.X = O.mat3Mul3(R, curX);
    S.dirty = true;
    syncFixChip();
  }
  function resetFix() {
    S.X = null;
    S.dirty = true;
    syncFixChip();
  }
  function correctedCols(cols) {
    return S.X ? O.mat3ApplyToCols(S.X, cols) : cols;
  }

  // ---------- 渲染 ----------
  function render() {
    var ang = displayAngles();
    var frame = deviceFrame(ang.a, ang.b, ang.g);
    if (!frame) return;
    var F = correctedCols(frame.css);
    var m3 = 'translate(-50%,-50%) ' + O.cssMatrix3dFromCssAxes(F);
    el.phone.style.transform = m3;
    el.axismodel.style.transform = m3;

    // 光照: 前-上-右打光; css 空间 y 向下 → 上 = -y
    var L = norm3([0.55, -0.5, 0.85]);
    var faces = [
      { e: el.shadeFront, n: F.z, d: 1 }, { e: el.shadeBack, n: F.z, d: -1 },
      { e: el.shadeRight, n: F.x, d: 1 }, { e: el.shadeLeft, n: F.x, d: -1 },
      { e: el.shadeTop, n: F.y, d: 1 }, { e: el.shadeBottom, n: F.y, d: -1 }
    ];
    for (var i = 0; i < faces.length; i++) {
      var nd = dot3(faces[i].n, L) * faces[i].d;
      faces[i].e.style.opacity = clamp(0.72 - 0.72 * Math.max(0, nd), 0, 0.8).toFixed(3);
    }

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
    if (ang.g >= 0) { el.fillG.style.left = '50%'; el.fillG.style.width = gp + '%'; }
    else { el.fillG.style.left = (50 + gp) + '%'; el.fillG.style.width = (-gp) + '%'; }
    var n = el.compassSvg.querySelector('#needle');
    if (n) n.setAttribute('transform', 'rotate(' + (-ang.a) + ' 50 50)');
  }
  function cardinal(deg) {
    var names = ['北', '东北', '东', '东南', '南', '西南', '西', '西北'];
    return names[Math.round(wrap360(deg) / 45) % 8];
  }
  function renderPoseText(ang) {
    var b = ang.b, g = ang.g, tags = [];
    if (b < 18) tags.push({ t: '近平放·屏朝上', c: 'var(--a)' });
    else if (b > 162) tags.push({ t: '近平放·屏朝下', c: 'var(--g)' });
    else if (b >= 68 && b <= 112) tags.push({ t: '竖立', c: 'var(--ok)' });
    else tags.push({ t: b < 68 ? '倾斜·偏平放' : '倾斜·偏翻转', c: 'var(--b)' });
    if (g > 8) tags.push({ t: '右倾 ' + fmt(g) + '°', c: 'var(--g)' });
    else if (g < -8) tags.push({ t: '左倾 ' + fmt(Math.abs(g)) + '°', c: 'var(--g)' });
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
    var ang = displayAngles();
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
      ctx.fillStyle = 'rgba(255,255,255,.14)';
      ctx.font = '11px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('等待数据…', w / 2, h / 2);
      ctx.textAlign = 'left';
      return;
    }
    var padL = 6, padR = 8, padT = 4, padB = 6;
    var lane = (h - padT - padB) / 3;
    var series = [
      { d: c.a, min: 0, max: 360, col: '#4cc9f0' },
      { d: c.b, min: 0, max: 180, col: '#ffb84d' },
      { d: c.g, min: -90, max: 90, col: '#ff6b9d' }
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
      ctx.strokeStyle = ser.col;
      ctx.lineWidth = 1.6;
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
      var lx = xAt(t1), ly = y1 - 5 - ((last - ser.min) / (ser.max - ser.min)) * (lane - 10);
      ctx.fillStyle = ser.col;
      ctx.beginPath(); ctx.arc(lx, ly, 2, 0, Math.PI * 2); ctx.fill();
    }
  }

  // ---------- 罗盘 ----------
  function buildCompass() {
    var ns = 'http://www.w3.org/2000/svg';
    var svg = el.compassSvg;
    var g = document.createElementNS(ns, 'g');
    g.setAttribute('id', 'needle');
    var tri = document.createElementNS(ns, 'polygon');
    tri.setAttribute('points', '50,14 57,40 43,40');
    tri.setAttribute('fill', 'rgba(76,201,240,.92)');
    tri.setAttribute('stroke', '#0a0e16');
    g.appendChild(tri);
    var pin = document.createElementNS(ns, 'circle');
    pin.setAttribute('cx', '50'); pin.setAttribute('cy', '50'); pin.setAttribute('r', '3');
    pin.setAttribute('fill', '#fff');
    g.appendChild(pin);
    svg.appendChild(g);
    for (var k = 0; k < 24; k++) {
      var a1 = k * 15 - 90;
      var r1 = 43, r2 = (k % 6 === 0) ? 38 : 40;
      var line = document.createElementNS(ns, 'line');
      line.setAttribute('x1', 50 + r1 * Math.cos(a1 * Math.PI / 180));
      line.setAttribute('y1', 50 + r1 * Math.sin(a1 * Math.PI / 180));
      line.setAttribute('x2', 50 + r2 * Math.cos(a1 * Math.PI / 180));
      line.setAttribute('y2', 50 + r2 * Math.sin(a1 * Math.PI / 180));
      line.setAttribute('stroke', k % 6 === 0 ? 'rgba(255,255,255,.75)' : 'rgba(255,255,255,.28)');
      line.setAttribute('stroke-width', k % 6 === 0 ? 1.6 : 0.8);
      svg.appendChild(line);
    }
    var letters = [['N', -90, 32], ['E', 0, 45], ['S', 90, 45], ['W', 180, 45]];
    letters.forEach(function (lt) {
      var rad = lt[1] * Math.PI / 180;
      var txt = document.createElementNS(ns, 'text');
      txt.setAttribute('x', 50 + 32 * Math.cos(rad));
      txt.setAttribute('y', 50 + 32 * Math.sin(rad) + 3);
      txt.setAttribute('text-anchor', 'middle');
      txt.setAttribute('font-size', '8.5');
      txt.setAttribute('fill', '#8fa0b8');
      txt.textContent = lt[0];
      svg.appendChild(txt);
    });
  }

  // ---------- 视角 ----------
  function applyCam() {
    el.orbit.style.transform = 'rotateX(' + CAM.pitch + 'deg) rotateY(' + CAM.yaw + 'deg) translateZ(' + CAM.zoom + 'px)';
  }
  function camReset() { CAM.pitch = -14; CAM.yaw = 18; CAM.zoom = -20; applyCam(); }
  function bindCam() {
    var drag = null;
    el.scene.addEventListener('pointerdown', function (e) {
      drag = { x: e.clientX, y: e.clientY, moved: false };
      try { el.scene.setPointerCapture(e.pointerId); } catch (err) {}
      el.scene.classList.add('dragging');
    });
    el.scene.addEventListener('pointermove', function (e) {
      if (!drag) return;
      var dx = e.clientX - drag.x, dy = e.clientY - drag.y;
      if (Math.abs(dx) + Math.abs(dy) > 3) drag.moved = true;
      CAM.yaw = CAM.yaw + dx * 0.32;
      CAM.pitch = clamp(CAM.pitch - dy * 0.32, -88, 40);
      drag.x = e.clientX; drag.y = e.clientY;
      applyCam();
    });
    function endDrag(e) {
      if (drag && !drag.moved) camReset();
      drag = null;
      el.scene.classList.remove('dragging');
      try { el.scene.releasePointerCapture(e.pointerId); } catch (err) {}
    }
    el.scene.addEventListener('pointerup', endDrag);
    el.scene.addEventListener('pointercancel', function () { drag = null; el.scene.classList.remove('dragging'); });
    el.scene.addEventListener('wheel', function (e) {
      e.preventDefault();
      CAM.zoom = clamp(CAM.zoom + e.deltaY * 0.6, -850, 460);
      applyCam();
    }, { passive: false });
  }

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
        ingest(parseFloat(el.simA.value), parseFloat(el.simB.value), parseFloat(el.simG.value), 'sim');
      });
    });
    el.btnZero.addEventListener('click', function () {
      stopDemo();
      el.simA.value = 0; el.simB.value = 0; el.simG.value = 0;
      if (S.mode !== 'sim') setMode('sim');
      ingest(0, 0, 0, 'sim');
    });
    el.btnDemo.addEventListener('click', function () {
      if (S.demo) { stopDemo(); return; }
      if (S.mode !== 'sim') setMode('sim');
      S.demo = true; S.demoStart = performance.now();
      el.btnDemo.textContent = '⏸ 停止演示';
    });
  }

  // ---------- 校准/平滑控件 ----------
  function bindTools() {
    el.btnCalib.addEventListener('click', calibToCurrent);
    el.btnSpinL.addEventListener('click', function () { spinModel('L'); });
    el.btnSpinR.addEventListener('click', function () { spinModel('R'); });
    el.btnCamReset.addEventListener('click', function () { resetFix(); camReset(); });
    el.smoothR.addEventListener('input', function () {
      var v = parseInt(el.smoothR.value, 10);
      S.smooth = v / 100;
      el.smVal.textContent = v;
    });
  }

  // ---------- Hz / 状态 ----------
  function syncChip() {
    el.chipAbs.textContent = S.absolute === null ? '—' : (S.absolute ? '绝对(罗盘)' : '相对');
    if (S.mode === 'sensor' && S.sensorState === 'running') el.dotLive.className = 'dot on';
  }
  function hzLoop() {
    var now = performance.now();
    if (now - S.evWinStart >= 1000) {
      S.hzShown = Math.round(S.evCount * 1000 / (now - S.evWinStart));
      S.evCount = 0; S.evWinStart = now;
      el.chipHz.textContent = S.hzShown + ' Hz';
      if (S.mode === 'sensor' && S.sensorState === 'running' && now - S.lastEvT > 2500) setSensorState('lost');
    }
    setTimeout(hzLoop, 250);
  }

  // ---------- 尺寸 ----------
  function bindResize() {
    function sizePhone() {
      var w = el.scene.clientWidth, h = el.scene.clientHeight;
      var pw = clamp(Math.min(w * 0.30, h * 0.42), 56, 170);
      var ph = pw * 2.15, pd = pw * 0.135;
      var st = { '--pw': pw.toFixed(1) + 'px', '--ph': ph.toFixed(1) + 'px', '--pd': pd.toFixed(1) + 'px' };
      for (var k in st) {
        el.phone.style.setProperty(k, st[k]);
        el.axismodel.style.setProperty(k, st[k]);
      }
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
    var dt = clamp((now - S.lastTick) / 1000, 0.001, 0.1);
    S.lastTick = now;

    if (S.demo) {
      var t = (now - S.demoStart) / 1000;
      ingest(wrap360(150 + t * 26), clamp(90 + 65 * Math.sin(t * 0.6), 0, 180), clamp(55 * Math.sin(t * 0.9), -90, 90), 'demo');
      el.simA.value = S.rawA; el.simB.value = S.rawB.toFixed(1); el.simG.value = S.rawG.toFixed(1);
    } else {
      stepSmooth(dt);
    }
    if (S.dirty && now - lastRenderT > 16) {
      S.dirty = false; lastRenderT = now;
      render();
    }
    if (now - lastChartT > 400) { drawChart(); lastChartT = now; }
  }

  // ---------- UI ----------
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

  // ---------- 启动 ----------
  function boot() {
    buildCompass();
    bindCam(); bindSim(); bindTools(); bindUI(); bindResize();
    el.simA.value = 0; el.simB.value = 90; el.simG.value = 0;
    ingest(0, 90, 0, 'sim');
    syncChip(); paintSeg(); syncFixChip(); camReset();

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
