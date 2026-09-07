/* main.js — 陀螺仪姿态可视化主逻辑(经典脚本, 无模块依赖) */
(function () {
  'use strict';

  var O = (window.__gyroOrient || {}).deviceFrameFromAngles;
  if (!O) { console.error('orient.js 未加载'); return; }

  // ---------- 小工具 ----------
  function $(id) { return document.getElementById(id); }
  function clamp(v, a, b) { return v < a ? a : (v > b ? b : v); }
  function fmt(v, d) { d = d == null ? 1 : d; var s = v.toFixed(d); return (v < 0 && v > -0.05 ? '0' : s); }
  function wrap360(x) { x = x % 360; return x < 0 ? x + 360 : x; }
  function norm3(v) { var l = Math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]) || 1; return [v[0]/l, v[1]/l, v[2]/l]; }
  function dot3(a, b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }

  var MOBILE = (typeof matchMedia === 'function' && matchMedia('(pointer:coarse)').matches) ||
               /iPhone|iPad|iPod|Android|Mobile/i.test(navigator.userAgent || '');
  var NEED_GESTURE = typeof window !== 'undefined' && 'DeviceOrientationEvent' in window &&
                     typeof window.DeviceOrientationEvent.requestPermission === 'function';
  var SENSOR_SUPPORTED = 'DeviceOrientationEvent' in window;

  // ---------- DOM ----------
  var el = {
    dotLive: $('dotLive'), chipMode: $('chipMode'), chipHz: $('chipHz'), chipAbs: $('chipAbs'),
    phone: $('phone'), orbit: $('orbit'), scene: $('scene'),
    shadeFront: document.querySelector('#phone .f-front .shade'),
    shadeBack: document.querySelector('#phone .f-back .shade'),
    shadeRight: document.querySelector('#phone .f-right .shade'),
    shadeLeft: document.querySelector('#phone .f-left .shade'),
    shadeTop: document.querySelector('#phone .f-top .shade'),
    shadeBottom: document.querySelector('#phone .f-bottom .shade'),
    vA: $('vA'), vB: $('vB'), vG: $('vG'),
    bvA: $('bvA'), bvB: $('bvB'), bvG: $('bvG'),
    fillA: $('fillA'), fillB: $('fillB'), fillG: $('fillG'),
    poseLine: $('poseLine'), poseWord: $('poseWord'),
    msgSensor: $('msgSensor'), msgSensorText: $('msgSensorText'),
    msgError: $('msgError'), msgErrorText: $('msgErrorText'), btnEnable: $('btnEnable'),
    simA: $('simA'), simB: $('simB'), simG: $('simG'),
    simAv: $('simAv'), simBv: $('simBv'), simGv: $('simGv'),
    btnDemo: $('btnDemo'), btnZero: $('btnZero'),
    chart: $('chart'), chartPause: $('chartPause'), segMode: $('segMode'),
    compassSvg: $('compassSvg'), helpOv: $('helpOv')
  };

  // ---------- 状态 ----------
  var S = {
    mode: 'sim',                 // 'sim' | 'sensor'
    sensorState: 'idle',         // idle|wait|running|denied|unsupported|lost
    absolute: null,              // null 未知 / true 绝对 / false 相对
    a: 0, b: 90, g: 0,           // 当前展示角度(度)
    dirty: false,
    demo: false, demoStart: 0,
    chartPaused: false,
    chart: { t: [], a: [], b: [], g: [] },
    lastSample: 0,
    evCount: 0, evWinStart: performance.now(), hzShown: 0,
    lastEvT: 0
  };
  var CAM = { pitch: -22, yaw: 14, zoom: -40 };

  // ---------- 传感器接入 ----------
  var list = { rel: null, abs: null };
  function onOrient(e) {
    var a = e.alpha, b = e.beta, g = e.gamma;
    if (a == null || b == null || g == null) return;
    if (!isFinite(a + b + g)) return;
    S.absolute = !!e.absolute && e.absolute !== null;
    el.chipAbs.textContent = S.absolute ? '绝对' : '相对';
    S.evCount++;
    S.lastEvT = performance.now();
    setPose(wrap360(a), clamp(b, -180, 180), clamp(g, -90, 90));
  }
  function attachSensor() {
    if (!SENSOR_SUPPORTED) return false;
    window.addEventListener('deviceorientation', onOrient, true);
    if ('ondeviceorientationabsolute' in window) {
      window.addEventListener('deviceorientationabsolute', onOrient, true);
    }
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
      // iOS 13+: requestPermission 必须在用户手势回调中调用
      if (!fromGesture) { setSensorState('wait'); return; }
      setSensorState('wait');
      try {
        var p = window.DeviceOrientationEvent.requestPermission();
        if (p && typeof p.then === 'function') {
          p.then(function (res) {
            if (res === 'granted') { attachSensor(); setSensorState('running'); }
            else setSensorState('denied');
          }).catch(function () { setSensorState('denied'); });
        } else {
          attachSensor(); setSensorState('running');
        }
      } catch (err) { setSensorState('denied'); }
    } else {
      attachSensor();
      setSensorState('running');
    }
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
      case 'running':
        el.chipMode.textContent = '传感器·运行中';
        break;
      case 'wait':
        el.chipMode.textContent = '传感器·待授权';
        el.msgSensorText.textContent = 'iOS 需要先授权：点击下方/顶部的「传感器」后，在系统弹窗选择“允许”。';
        el.msgSensor.classList.add('show');
        break;
      case 'denied':
        el.chipMode.textContent = '传感器·已拒绝';
        el.msgErrorText.textContent = '传感器权限被拒绝。请检查：页面需 HTTPS 或 localhost；iOS 需在 Safari 中打开并在系统设置允许“运动与健身”。可切回模拟模式继续体验。';
        el.msgError.classList.add('show');
        break;
      case 'unsupported':
        el.chipMode.textContent = '传感器·不支持';
        el.msgErrorText.textContent = '当前浏览器不支持 DeviceOrientation，或不在安全上下文(需 HTTPS/localhost)。已自动使用模拟模式。';
        el.msgError.classList.add('show');
        break;
      case 'lost':
        el.dotLive.className = 'dot warn';
        el.chipMode.textContent = '传感器·无数据';
        el.btnEnable.style.display = '';
        el.msgErrorText.textContent = '传感器一直没有数据。可点击重新尝试授权，或切换模拟模式。';
        el.msgError.classList.add('show');
        break;
      case 'idle':
        el.chipMode.textContent = '传感器·空闲';
        break;
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
      if (!SENSOR_SUPPORTED) { tryEnableSensor(); return; }
      tryEnableSensor(fromGesture);
    } else {
      detachSensor();
      S.absolute = null;
      el.dotLive.className = 'dot';
      el.msgSensor.classList.remove('show');
      el.msgError.classList.remove('show');
      el.btnEnable.style.display = 'none';
      el.chipMode.textContent = '模拟模式';
      S.sensorState = 'idle';
      syncChip();
    }
  }

  // ---------- 姿态设置(唯一入口) ----------
  function setPose(aDeg, bDeg, gDeg) {
    S.a = aDeg; S.b = bDeg; S.g = gDeg;
    S.dirty = true;
  }

  // ---------- 渲染 ----------
  function render() {
    var f = O(S.a, S.b, S.g);
    if (!f) return;
    var m3 = 'translate(-50%,-50%) ' + (window.__gyroOrient.cssMatrix3dFromCssAxes(f.css));
    el.phone.style.transform = m3;

    // 光照(世界空间): 从“前-上-右”方向打光; css 空间 y 向下, 上=-y
    var L = norm3([0.55, -0.55, 0.8]);
    var faces = [
      { e: el.shadeFront, n: f.css.z, d: 1 }, { e: el.shadeBack, n: f.css.z, d: -1 },
      { e: el.shadeRight, n: f.css.x, d: 1 }, { e: el.shadeLeft, n: f.css.x, d: -1 },
      { e: el.shadeTop, n: f.css.y, d: 1 }, { e: el.shadeBottom, n: f.css.y, d: -1 }
    ];
    for (var i = 0; i < faces.length; i++) {
      var nd = dot3(faces[i].n, L) * faces[i].d;
      var sh = clamp(0.78 - 0.78 * Math.max(0, nd), 0, 0.85);
      faces[i].e.style.opacity = sh.toFixed(3);
    }

    renderNumbers();
    renderBars();
    renderPoseText();
    sampleChart();
  }

  function renderNumbers() {
    el.vA.textContent = fmt(S.a); el.vB.textContent = fmt(S.b); el.vG.textContent = fmt(S.g);
    el.bvA.textContent = fmt(S.a) + '°'; el.bvB.textContent = fmt(S.b) + '°'; el.bvG.textContent = fmt(S.g) + '°';
    el.simAv.textContent = fmt(parseFloat(el.simA.value)) + '°';
    el.simBv.textContent = fmt(parseFloat(el.simB.value)) + '°';
    el.simGv.textContent = fmt(parseFloat(el.simG.value)) + '°';
  }
  function renderBars() {
    var a = S.a / 360 * 100, b = S.b / 180 * 100;
    el.fillA.style.width = clamp(a, 0, 100) + '%';
    el.fillB.style.width = clamp(b, 0, 100) + '%';
    var gp = S.g / 180 * 100;
    if (S.g >= 0) { el.fillG.style.left = '50%'; el.fillG.style.width = gp + '%'; }
    else { el.fillG.style.left = (50 + gp) + '%'; el.fillG.style.width = (-gp) + '%'; }
    el.compassSvg.querySelector('#needle').setAttribute('transform', 'rotate(' + (-S.a) + ' 50 50)');
  }
  function cardinal(deg) {
    var names = ['北', '东北', '东', '东南', '南', '西南', '西', '西北'];
    return names[Math.round(wrap360(deg) / 45) % 8];
  }
  function poseTags() {
    var tags = [];
    var b = S.b, g = S.g;
    if (b < 18) tags.push({ t: '近平放·屏朝上', c: 'var(--a)' });
    else if (b > 162) tags.push({ t: '近平放·屏朝下', c: 'var(--g)' });
    else if (b >= 68 && b <= 112) tags.push({ t: '竖立', c: 'var(--ok)' });
    else tags.push({ t: b < 68 ? '倾斜·偏平放' : '倾斜·偏翻转', c: 'var(--b)' });
    if (g > 8) tags.push({ t: '右倾 ' + fmt(g) + '°', c: 'var(--g)' });
    else if (g < -8) tags.push({ t: '左倾 ' + fmt(Math.abs(g)) + '°', c: 'var(--g)' });
    else tags.push({ t: '左右端正', c: 'var(--b)' });
    return tags;
  }
  function renderPoseText() {
    var tags = poseTags();
    el.poseLine.innerHTML = '';
    tags.forEach(function (tg) {
      var s = document.createElement('span');
      s.className = 'tag';
      s.style.borderColor = tg.c; s.style.color = tg.c;
      s.textContent = tg.t;
      el.poseLine.appendChild(s);
    });
    var hint = tags[0].t + (S.b > 18 && S.b < 162 ? ' · ' + tags[2].t : '');
    el.poseWord.textContent = hint + ' · α ' + fmt(S.a) + '°';
  }

  // ---------- 曲线 ----------
  function sampleChart() {
    var now = performance.now();
    if (S.chartPaused) return;
    if (now - S.lastSample < 33) return;   // ~30Hz 采样
    S.lastSample = now;
    var c = S.chart;
    c.t.push(now); c.a.push(S.a); c.b.push(S.b); c.g.push(S.g);
    var span = 30000;
    while (c.t.length && now - c.t[0] > span) {
      c.t.shift(); c.a.shift(); c.b.shift(); c.g.shift();
    }
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
    if (!c.t.length) { drawEmpty(ctx, w, h); return; }
    var padL = 6, padR = 8, padT = 4, padB = 6;
    var lane = (h - padT - padB) / 3;
    var series = [
      { d: c.a, min: 0, max: 360, col: '#4cc9f0', name: 'α' },
      { d: c.b, min: 0, max: 180, col: '#ffb84d', name: 'β' },
      { d: c.g, min: -90, max: 90, col: '#ff6b9d', name: 'γ' }
    ];
    ctx.font = '9px sans-serif';
    // 时间轴
    var t0 = c.t[0], t1 = c.t[c.t.length - 1];
    var tspan = Math.max(t1 - t0, 1000);
    function xAt(t) {
      return padL + (w - padL - padR) * (1 - (t1 - t) / tspan);
    }
    for (var s = 0; s < 3; s++) {
      var y0 = padT + lane * s, y1 = y0 + lane;
      // 网格
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
        var n = (v - ser.min) / (ser.max - ser.min);
        var px = xAt(c.t[i]);
        var py = y1 - 5 - n * (lane - 10);
        if (!started) { ctx.moveTo(px, py); started = true; }
        else ctx.lineTo(px, py);
      }
      ctx.stroke();
      // 末点
      var last = ser.d[ser.d.length - 1];
      var lx = xAt(t1), ly = y1 - 5 - ((last - ser.min) / (ser.max - ser.min)) * (lane - 10);
      ctx.fillStyle = ser.col;
      ctx.beginPath(); ctx.arc(lx, ly, 2, 0, Math.PI * 2); ctx.fill();
    }
  }
  function drawEmpty(ctx, w, h) {
    ctx.fillStyle = 'rgba(255,255,255,.14)';
    ctx.font = '11px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('等待数据…', w / 2, h / 2);
    ctx.textAlign = 'left';
  }

  // ---------- 罗盘绘制(一次性) ----------
  function buildCompass() {
    var ns = 'http://www.w3.org/2000/svg';
    var svg = el.compassSvg;
    var g = document.createElementNS(ns, 'g');
    g.setAttribute('id', 'needle');
    // 指针: 指向设备顶部的三角(按 -α 旋转; α 逆时针为正, 屏幕坐标顺时针为负)
    var tri = document.createElementNS(ns, 'polygon');
    tri.setAttribute('points', '50,14 57,40 43,40');
    tri.setAttribute('fill', 'rgba(76,201,240,.9)');
    tri.setAttribute('stroke', '#0a0e16');
    g.appendChild(tri);
    var pin = document.createElementNS(ns, 'circle');
    pin.setAttribute('cx', '50'); pin.setAttribute('cy', '50'); pin.setAttribute('r', '3');
    pin.setAttribute('fill', '#fff');
    g.appendChild(pin);
    svg.appendChild(g);
    // 刻度与方位
    for (var k = 0; k < 24; k++) {
      var a1 = k * 15 - 90;
      var r1 = 45, r2 = (k % 6 === 0) ? 40 : 42;
      var x1 = 50 + r1 * Math.cos(a1 * Math.PI / 180), y1 = 50 + r1 * Math.sin(a1 * Math.PI / 180);
      var x2 = 50 + r2 * Math.cos(a1 * Math.PI / 180), y2 = 50 + r2 * Math.sin(a1 * Math.PI / 180);
      var line = document.createElementNS(ns, 'line');
      line.setAttribute('x1', x1); line.setAttribute('y1', y1);
      line.setAttribute('x2', x2); line.setAttribute('y2', y2);
      line.setAttribute('stroke', k % 6 === 0 ? 'rgba(255,255,255,.75)' : 'rgba(255,255,255,.28)');
      line.setAttribute('stroke-width', k % 6 === 0 ? 1.6 : 0.8);
      svg.appendChild(line);
    }
    var letters = [['N', -90, 34], ['E', 0, 47], ['S', 90, 47], ['W', 180, 47]];
    letters.forEach(function (lt) {
      var rad = (lt[1]) * Math.PI / 180;
      var txt = document.createElementNS(ns, 'text');
      txt.setAttribute('x', 50 + 33 * Math.cos(rad));
      txt.setAttribute('y', 50 + 33 * Math.sin(rad) + 3);
      txt.setAttribute('text-anchor', 'middle');
      txt.setAttribute('font-size', '8.5');
      txt.setAttribute('fill', '#8fa0b8');
      txt.textContent = lt[0];
      svg.appendChild(txt);
    });
  }

  // ---------- 视角控制 ----------
  function applyCam() {
    el.orbit.style.transform = 'rotateX(' + CAM.pitch + 'deg) rotateY(' + CAM.yaw + 'deg) translateZ(' + CAM.zoom + 'px)';
  }
  function camReset() { CAM.pitch = -22; CAM.yaw = 14; CAM.zoom = -40; applyCam(); }
  function bindCam() {
    var drag = null;
    el.scene.addEventListener('pointerdown', function (e) {
      drag = { x: e.clientX, y: e.clientY, moved: false };
      el.scene.setPointerCapture(e.pointerId);
      el.scene.classList.add('dragging');
    });
    el.scene.addEventListener('pointermove', function (e) {
      if (!drag) return;
      var dx = e.clientX - drag.x, dy = e.clientY - drag.y;
      if (Math.abs(dx) + Math.abs(dy) > 3) drag.moved = true;
      CAM.yaw = CAM.yaw + dx * 0.32;
      CAM.pitch = clamp(CAM.pitch - dy * 0.32, -88, 30);
      drag.x = e.clientX; drag.y = e.clientY;
      applyCam();
    });
    el.scene.addEventListener('pointerup', function (e) {
      if (drag && drag.moved === false) camReset();
      drag = null;
      el.scene.classList.remove('dragging');
      try { el.scene.releasePointerCapture(e.pointerId); } catch (err) {}
    });
    el.scene.addEventListener('pointercancel', function () {
      drag = null; el.scene.classList.remove('dragging');
    });
    el.scene.addEventListener('wheel', function (e) {
      e.preventDefault();
      CAM.zoom = clamp(CAM.zoom + e.deltaY * 0.6, -800, 420);
      applyCam();
    }, { passive: false });
  }

  // ---------- 模拟控制 ----------
  function applySimFromSliders() {
    var a = parseFloat(el.simA.value), b = parseFloat(el.simB.value), g = parseFloat(el.simG.value);
    if (S.mode !== 'sim' && !S.demo) setMode('sim');
    setPose(a, b, g);
  }
  function bindSim() {
    [el.simA, el.simB, el.simG].forEach(function (inp) {
      inp.addEventListener('input', function () {
        if (S.demo) stopDemo();
        applySimFromSliders();
      });
    });
    el.btnZero.addEventListener('click', function () {
      stopDemo();
      el.simA.value = 0; el.simB.value = 0; el.simG.value = 0;
      if (S.mode !== 'sim') setMode('sim');
      setPose(0, 0, 0);
    });
    el.btnDemo.addEventListener('click', toggleDemo);
  }
  function stopDemo() {
    if (!S.demo) return;
    S.demo = false;
    el.btnDemo.textContent = '▶ 演示摇摆';
  }
  function toggleDemo() {
    if (S.demo) { stopDemo(); return; }
    if (S.mode !== 'sim') setMode('sim');
    S.demo = true;
    S.demoStart = performance.now();
    el.btnDemo.textContent = '⏸ 停止演示';
  }

  // ---------- Hz / 状态 ----------
  function syncChip() {
    el.chipAbs.textContent = S.absolute === null ? '—'
      : (S.absolute ? '绝对(罗盘)' : '相对');
    if (S.mode === 'sensor') {
      if (S.sensorState === 'running') el.dotLive.className = 'dot on';
      else if (S.sensorState === 'wait') el.dotLive.className = 'dot warn';
    }
  }
  function hzLoop() {
    var now = performance.now();
    if (now - S.evWinStart >= 1000) {
      S.hzShown = Math.round(S.evCount * 1000 / (now - S.evWinStart));
      S.evCount = 0; S.evWinStart = now;
      el.chipHz.textContent = S.hzShown + ' Hz';
      // 传感器存活检测
      if (S.mode === 'sensor' && S.sensorState === 'running' && now - S.lastEvT > 2500) {
        setSensorState('lost');
      }
    }
    setTimeout(hzLoop, 250);
  }

  // ---------- 图表尺寸 ----------
  var ro = null;
  function bindResize() {
    function sizePhone() {
      var scene = el.scene;
      var w = scene.clientWidth, h = scene.clientHeight;
      var pw = clamp(Math.min(w * 0.26, h * 0.34), 60, 150);
      var ph = pw * 2.15, pd = pw * 0.135;
      el.phone.style.setProperty('--pw', pw.toFixed(1) + 'px');
      el.phone.style.setProperty('--ph', ph.toFixed(1) + 'px');
      el.phone.style.setProperty('--pd', pd.toFixed(1) + 'px');
    }
    function sizeCanvas() { drawChart(); }
    if ('ResizeObserver' in window) {
      ro = new ResizeObserver(function () { sizePhone(); sizeCanvas(); });
      ro.observe(el.scene); ro.observe($('chartWrap'));
    } else {
      window.addEventListener('resize', function () { sizePhone(); sizeCanvas(); });
    }
    sizePhone();
  }

  // ---------- 主循环 ----------
  var lastCamT = 0, lastRenderT = 0;
  function tick(now) {
    requestAnimationFrame(tick);
    if (S.demo) {
      var dt = (now - S.demoStart) / 1000;
      var a = wrap360(150 + dt * 26);
      var b = 90 + 65 * Math.sin(dt * 0.6);
      var g = 55 * Math.sin(dt * 0.9);
      setPose(a, clamp(b, 0, 180), clamp(g, -90, 90));
      el.simA.value = a; el.simB.value = b.toFixed(1); el.simG.value = g.toFixed(1);
    }
    if (S.dirty && now - lastRenderT > 16) {
      S.dirty = false; lastRenderT = now;
      render();
    }
    if (now - lastCamT > 500) { drawChart(); lastCamT = now; }
  }

  // ---------- 帮助 / 分段 ----------
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
    document.addEventListener('keydown', function (e) {
      if (e.key === 'Escape') el.helpOv.classList.remove('show');
    });
  }

  // ---------- 启动 ----------
  function boot() {
    buildCompass();
    bindCam(); bindSim(); bindUI(); bindResize();
    el.simA.value = 0; el.simB.value = 90; el.simG.value = 0;
    setPose(0, 90, 0);
    syncChip();
    paintSeg();
    camReset();

    var wantSensor = SENSOR_SUPPORTED && MOBILE;
    if (wantSensor && NEED_GESTURE) {
      // iOS: 置为待授权, 等用户点击(不可自动弹授权)
      S.mode = 'sensor';
      paintSeg();
      setSensorState('wait');
    } else if (wantSensor) {
      setMode('sensor');   // Android 等: 自动挂载监听
    } else if (!SENSOR_SUPPORTED) {
      setMode('sim');
      showUnsupported();
    } else {
      // 桌面无陀螺仪 → 模拟模式(提示可切传感器)
      setMode('sim');
      el.chipMode.textContent = '模拟模式 · 桌面无陀螺仪';
    }

    hzLoop();
    requestAnimationFrame(tick);
    drawChart();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', boot);
  } else boot();
})();
