/**
 * PENDULARM Studio - Frontend Application
 * Connects via WebSocket to Rosbridge Gateway (127.0.0.1:9095)
 */

// ================= CONSTANTS & STATE =================
const WS_URL = `ws://${window.location.host}/ws`;

const state = {
  connected: false,
  linkCount: 2,
  params: {
    lengths: [1.0, 1.0],
    masses: [1.0, 1.0],
    gravity: 9.81,
  },
  integrator: 'rk4',
  timestep: 0.005,
  paused: false,
  simTime: 0.0,
  
  // Plant State
  q: [0.0, 0.0],
  qdot: [0.0, 0.0],
  effort: [0.0, 0.0],
  
  // PID Controller
  pidEnabled: false,
  setpointPos: [0.0, 0.0],
  kp: [40.0, 40.0],
  ki: [8.0, 8.0],
  kd: [10.0, 10.0],
  
  // IK & Goal
  ikTarget: { x: 1.0, y: 0.5, phi: 0.0 },
  ikSolution: null,
  activeGoal: null, // { x, y, phi, epsilon, successHold, startTime, distance }
  lastActionResult: null,
  
  // Trial
  trial: {
    running: false,
    duration: 20.0,
    elapsed: 0.0,
    targetsReached: 0,
    target: null,
    error: null,
    actionStatus: 'idle',
  },
  
  // Visuals & Canvas
  view: {
    scale: 160, // pixels per meter
    offsetX: 0,
    offsetY: 0,
    isPanning: false,
    panStartX: 0,
    panStartY: 0,
    isDraggingTarget: false,
  },
  trail: [],
  trailEnabled: true,
  maxTrailLength: 350,
  
  // Telemetry History (for charts)
  history: {
    time: [],
    q: [[], [], []],
    qdot: [[], [], []],
    effort: [[], [], []],
    energyT: [],
    energyV: [],
    energyTotal: [],
  },
  maxHistoryPoints: 240,
  activeChartTab: 'angles',
  fps: 60,
  lastFrameTime: performance.now(),
  frameCount: 0,
  lastFpsUpdate: performance.now(),
};

// ================= ROSBRIDGE PROTOCOL CLIENT =================
class RosbridgeClient {
  constructor(url) {
    this.url = url;
    this.ws = null;
    this.callId = 1;
    this.pendingCalls = new Map();
    this.subscriptions = new Map();
    this.reconnectTimer = null;
  }

  connect() {
    clearTimeout(this.reconnectTimer);
    try {
      this.ws = new WebSocket(this.url);
    } catch (e) {
      this.scheduleReconnect();
      return;
    }

    this.ws.onopen = () => {
      state.connected = true;
      updateConnectionStatus(true);
      console.log('[Rosbridge] Connected to gateway');
      this.resubscribe();
      fetchInitialParams();
    };

    this.ws.onmessage = (event) => {
      try {
        const msg = JSON.parse(event.data);
        this.handleMessage(msg);
      } catch (e) {
        console.error('[Rosbridge] Error parsing incoming JSON:', e);
      }
    };

    this.ws.onclose = () => {
      state.connected = false;
      updateConnectionStatus(false);
      this.scheduleReconnect();
    };

    this.ws.onerror = () => {
      state.connected = false;
      updateConnectionStatus(false);
    };
  }

  scheduleReconnect() {
    clearTimeout(this.reconnectTimer);
    this.reconnectTimer = setTimeout(() => {
      this.connect();
    }, 1500);
  }

  send(obj) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(obj));
    }
  }

  subscribe(topic, callback) {
    this.subscriptions.set(topic, callback);
    this.send({ op: 'subscribe', topic });
  }

  resubscribe() {
    for (const topic of this.subscriptions.keys()) {
      this.send({ op: 'subscribe', topic });
    }
  }

  publish(topic, msg) {
    this.send({ op: 'publish', topic, msg });
  }

  callService(service, args = {}) {
    return new Promise((resolve, reject) => {
      const id = `call_${this.callId++}`;
      const timeout = setTimeout(() => {
        if (this.pendingCalls.has(id)) {
          this.pendingCalls.delete(id);
          reject(new Error(`Service call to ${service} timed out`));
        }
      }, 3000);

      this.pendingCalls.set(id, { resolve, reject, timeout });
      this.send({ op: 'call_service', id, service, args });
    });
  }

  handleMessage(msg) {
    if (msg.op === 'publish') {
      const cb = this.subscriptions.get(msg.topic);
      if (cb) cb(msg.msg);
    } else if (msg.op === 'service_response') {
      const id = msg.id;
      if (id && this.pendingCalls.has(id)) {
        const { resolve, timeout } = this.pendingCalls.get(id);
        clearTimeout(timeout);
        this.pendingCalls.delete(id);
        resolve(msg);
      }
    }
  }
}

const ros = new RosbridgeClient(WS_URL);

// ================= DOM ELEMENT REFERENCES =================
const elements = {
  connStatus: document.getElementById('connStatus'),
  btnPlayPause: document.getElementById('btnPlayPause'),
  playIcon: document.getElementById('playIcon'),
  btnStep: document.getElementById('btnStep'),
  btnReset: document.getElementById('btnReset'),
  btnToggleTrail: document.getElementById('btnToggleTrail'),
  btnClearTrail: document.getElementById('btnClearTrail'),
  btnMode2Link: document.getElementById('btnMode2Link'),
  btnMode3Link: document.getElementById('btnMode3Link'),
  
  // HUD
  hudSimTime: document.getElementById('hudSimTime'),
  hudIntegrator: document.getElementById('hudIntegrator'),
  hudDt: document.getElementById('hudDt'),
  hudFps: document.getElementById('hudFps'),
  hudEeX: document.getElementById('hudEeX'),
  hudEeY: document.getElementById('hudEeY'),
  hudEePhi: document.getElementById('hudEePhi'),
  hudPhiContainer: document.getElementById('hudPhiContainer'),
  hudEnergy: document.getElementById('hudEnergy'),
  
  // Floating Banner
  trialFloatingBanner: document.getElementById('trialFloatingBanner'),
  floatingBannerTitle: document.getElementById('floatingBannerTitle'),
  floatingBannerSubtitle: document.getElementById('floatingBannerSubtitle'),
  floatingBannerGauge: document.getElementById('floatingBannerGauge'),
  
  // Simulation Tab
  sliderTimestep: document.getElementById('sliderTimestep'),
  valTimestep: document.getElementById('valTimestep'),
  sliderGravity: document.getElementById('sliderGravity'),
  valGravity: document.getElementById('valGravity'),
  linkLengthSliders: document.getElementById('linkLengthSliders'),
  linkMassSliders: document.getElementById('linkMassSliders'),
  btnApplyParams: document.getElementById('btnApplyParams'),
  btnDropHorizontal: document.getElementById('btnDropHorizontal'),
  btnDropUpright: document.getElementById('btnDropUpright'),
  btnDropHanging: document.getElementById('btnDropHanging'),
  btnKickArm: document.getElementById('btnKickArm'),
  
  // PID Tab
  chkPidEnable: document.getElementById('chkPidEnable'),
  pidStatusBanner: document.getElementById('pidStatusBanner'),
  jointSetpointSliders: document.getElementById('jointSetpointSliders'),
  btnSendTrajectory: document.getElementById('btnSendTrajectory'),
  btnZeroSetpoints: document.getElementById('btnZeroSetpoints'),
  gainsContainer: document.getElementById('gainsContainer'),
  btnApplyGains: document.getElementById('btnApplyGains'),
  btnPresetStiff: document.getElementById('btnPresetStiff'),
  btnPresetSoft: document.getElementById('btnPresetSoft'),
  
  // IK Tab
  ikTargetX: document.getElementById('ikTargetX'),
  ikTargetY: document.getElementById('ikTargetY'),
  ikTargetPhiSlider: document.getElementById('ikTargetPhiSlider'),
  ikTargetPhiVal: document.getElementById('ikTargetPhiVal'),
  ikPhiRow: document.getElementById('ikPhiRow'),
  btnSolveIk: document.getElementById('btnSolveIk'),
  ikResultCard: document.getElementById('ikResultCard'),
  btnApplyIkToPid: document.getElementById('btnApplyIkToPid'),
  
  // Action Tab
  actionTargetX: document.getElementById('actionTargetX'),
  actionTargetY: document.getElementById('actionTargetY'),
  actionTargetPhi: document.getElementById('actionTargetPhi'),
  actionPhiRow: document.getElementById('actionPhiRow'),
  actionEpsilon: document.getElementById('actionEpsilon'),
  actionSuccessHold: document.getElementById('actionSuccessHold'),
  btnSendGoal: document.getElementById('btnSendGoal'),
  btnCancelGoal: document.getElementById('btnCancelGoal'),
  actionStatusBadge: document.getElementById('actionStatusBadge'),
  actionGoalId: document.getElementById('actionGoalId'),
  actionDistance: document.getElementById('actionDistance'),
  actionProgressBar: document.getElementById('actionProgressBar'),
  actionResultCard: document.getElementById('actionResultCard'),
  
  // Trial Tab
  trialDuration: document.getElementById('trialDuration'),
  trialEpsilon: document.getElementById('trialEpsilon'),
  trialHold: document.getElementById('trialHold'),
  btnStartTrial: document.getElementById('btnStartTrial'),
  btnSkipTrial: document.getElementById('btnSkipTrial'),
  btnStopTrial: document.getElementById('btnStopTrial'),
  trialScore: document.getElementById('trialScore'),
  trialElapsed: document.getElementById('trialElapsed'),
  trialProgressBar: document.getElementById('trialProgressBar'),
  trialTargetCoord: document.getElementById('trialTargetCoord'),
  trialError: document.getElementById('trialError'),
  trialActionState: document.getElementById('trialActionState'),
  
  // ODE Playground Tab
  odeExpression: document.getElementById('odeExpression'),
  odeT0: document.getElementById('odeT0'),
  odeX0: document.getElementById('odeX0'),
  odeV0: document.getElementById('odeV0'),
  odeDt: document.getElementById('odeDt'),
  odeMethod: document.getElementById('odeMethod'),
  btnOdeStep: document.getElementById('btnOdeStep'),
  btnOdeSimulate: document.getElementById('btnOdeSimulate'),
  odeResultCard: document.getElementById('odeResultCard'),
  
  // Canvas & Charts
  armCanvas: document.getElementById('armCanvas'),
  canvasWrapper: document.getElementById('canvasWrapper'),
  telemetryChart: document.getElementById('telemetryChart'),
  chartLegend: document.getElementById('chartLegend'),
  btnZoomIn: document.getElementById('btnZoomIn'),
  btnZoomOut: document.getElementById('btnZoomOut'),
  btnResetView: document.getElementById('btnResetView'),
};

const armCtx = elements.armCanvas.getContext('2d');
const chartCtx = elements.telemetryChart.getContext('2d');

// ================= KINEMATICS & DYNAMICS MATH =================
function forwardKinematics(q, lengths) {
  const n = lengths.length;
  const joints = [{ x: 0, y: 0 }];
  let currentX = 0;
  let currentY = 0;
  let phi = 0;
  
  for (let i = 0; i < n; i++) {
    phi += q[i] || 0;
    currentX += lengths[i] * Math.cos(phi);
    currentY += lengths[i] * Math.sin(phi);
    joints.push({ x: currentX, y: currentY, phi });
  }
  return { joints, ee: { x: currentX, y: currentY, phi } };
}

function computeEnergies(q, qdot, params) {
  const n = params.lengths.length;
  // World angles
  const phi = [];
  const phidot = [];
  let accumPhi = 0;
  let accumPhidot = 0;
  for (let i = 0; i < n; i++) {
    accumPhi += (q[i] || 0);
    accumPhidot += (qdot[i] || 0);
    phi.push(accumPhi);
    phidot.push(accumPhidot);
  }

  // Potential Energy V = g * sum_{j=0}^{n-1} l_j * sin(phi_j) * (0.5 * m_j + sum_{i=j+1}^{n-1} m_i)
  let V = 0;
  for (let j = 0; j < n; j++) {
    let outerMass = 0;
    for (let i = j + 1; i < n; i++) outerMass += params.masses[i];
    V += params.gravity * params.lengths[j] * Math.sin(phi[j]) * (0.5 * params.masses[j] + outerMass);
  }

  // Kinetic Energy T = 0.5 * phidot^T * A * phidot
  let T = 0;
  for (let j = 0; j < n; j++) {
    for (let k = 0; k < n; k++) {
      let A_jk = 0;
      if (j === k) {
        let outerMass = 0;
        for (let i = j + 1; i < n; i++) outerMass += params.masses[i];
        A_jk = params.lengths[j] * params.lengths[j] * ((1.0 / 3.0) * params.masses[j] + outerMass);
      } else {
        const max_jk = Math.max(j, k);
        let outerMass = 0;
        for (let i = max_jk + 1; i < n; i++) outerMass += params.masses[i];
        A_jk = params.lengths[j] * params.lengths[k] * Math.cos(phi[j] - phi[k]) * (0.5 * params.masses[max_jk] + outerMass);
      }
      T += 0.5 * phidot[j] * A_jk * phidot[k];
    }
  }

  return { T: Math.max(0, T), V, total: T + V };
}

// Client-side 2-link analytical IK for fast interactive ghost preview
function solve2LinkClient(l1, l2, x, y) {
  const r = Math.hypot(x, y);
  const outer = l1 + l2;
  const inner = Math.abs(l1 - l2);
  if (r > outer || r < inner) return null;
  const cosQ2 = Math.min(1.0, Math.max(-1.0, (r * r - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)));
  const q2 = Math.acos(cosQ2);
  const q1 = Math.atan2(y, x) - Math.atan2(l2 * Math.sin(q2), l1 + l2 * Math.cos(q2));
  return [q1, q2];
}

function solve3LinkClient(l1, l2, l3, x, y, phi) {
  const wx = x - l3 * Math.cos(phi);
  const wy = y - l3 * Math.sin(phi);
  const twoLinkSol = solve2LinkClient(l1, l2, wx, wy);
  if (!twoLinkSol) return null;
  const q3 = phi - twoLinkSol[0] - twoLinkSol[1];
  return [twoLinkSol[0], twoLinkSol[1], q3];
}

// ================= TOPICS & SERVICES HANDLERS =================
function subscribeTopics() {
  // /joint_states (published at ~60Hz)
  ros.subscribe('/joint_states', (msg) => {
    if (msg.position && Array.isArray(msg.position)) {
      const n = msg.position.length;
      if (n !== state.linkCount) {
        setLinkCount(n);
      }
      state.q = msg.position;
      state.qdot = msg.velocity || new Array(n).fill(0);
      state.effort = msg.effort || new Array(n).fill(0);
      
      if (msg.header && msg.header.stamp) {
        state.simTime = msg.header.stamp.sec + (msg.header.stamp.nanosec || 0) * 1e-9;
      }
      
      // Update motion trail
      const fk = forwardKinematics(state.q, state.params.lengths);
      if (state.trailEnabled) {
        state.trail.push({ x: fk.ee.x, y: fk.ee.y, time: state.simTime });
        if (state.trail.length > state.maxTrailLength) {
          state.trail.shift();
        }
      }
      
      // Compute energy
      const energy = computeEnergies(state.q, state.qdot, state.params);
      
      // Push history
      pushTelemetryHistory(state.simTime, state.q, state.qdot, state.effort, energy);
      
      // Update HUD values
      elements.hudSimTime.textContent = `${state.simTime.toFixed(3)} s`;
      elements.hudEeX.textContent = `${fk.ee.x.toFixed(3)} m`;
      elements.hudEeY.textContent = `${fk.ee.y.toFixed(3)} m`;
      elements.hudEePhi.textContent = `${(fk.ee.phi * 180 / Math.PI).toFixed(1)}°`;
      elements.hudEnergy.textContent = `${energy.total.toFixed(2)} J`;
    }
  });

  // /ik_action/feedback
  ros.subscribe('/ik_action/feedback', (msg) => {
    state.activeGoal = {
      id: msg.goal_id,
      distance: msg.distance_remaining,
      target: msg.target,
      elapsed: msg.elapsed,
      positions: msg.positions,
    };
    elements.actionGoalId.textContent = msg.goal_id || '—';
    elements.actionDistance.textContent = `${(msg.distance_remaining || 0).toFixed(4)} m`;
    elements.actionStatusBadge.textContent = 'ACTIVE';
    elements.actionStatusBadge.className = 'badge active';
    
    // Animate distance progress bar (assuming max span ~ 2.0m)
    const pct = Math.max(0, Math.min(100, (1.0 - (msg.distance_remaining / 1.5)) * 100));
    elements.actionProgressBar.style.width = `${pct}%`;
  });

  // /ik_action/result
  ros.subscribe('/ik_action/result', (msg) => {
    state.lastActionResult = msg;
    state.activeGoal = null;
    elements.actionStatusBadge.textContent = (msg.outcome || 'IDLE').toUpperCase();
    elements.actionStatusBadge.className = `badge ${msg.outcome || ''}`;
    elements.actionProgressBar.style.width = msg.outcome === 'reached' ? '100%' : '0%';
    
    elements.actionResultCard.innerHTML = `
      <div class="card-row">
        <span class="label">Goal:</span>
        <span class="val font-mono">${msg.goal_id}</span>
      </div>
      <div class="card-row">
        <span class="label">Outcome:</span>
        <span class="badge ${msg.outcome}">${msg.outcome}</span>
      </div>
      <div class="card-row">
        <span class="label">Final Distance:</span>
        <span class="val font-mono">${(msg.final_distance || 0).toFixed(5)} m</span>
      </div>
    `;
  });

  // /ik_trial/status (published at ~60Hz)
  ros.subscribe('/ik_trial/status', (msg) => {
    state.trial = {
      running: msg.running || false,
      duration: msg.duration || 20.0,
      elapsed: msg.elapsed || 0.0,
      targetsReached: msg.targets_reached || 0,
      target: msg.target,
      error: msg.error,
      actionStatus: msg.action_status || 'idle',
      desiredPositions: msg.desired_positions,
    };

    // Update scoreboard
    elements.trialScore.textContent = state.trial.targetsReached;
    elements.trialElapsed.textContent = `${state.trial.elapsed.toFixed(1)} / ${state.trial.duration.toFixed(1)} s`;
    const trialPct = Math.min(100, (state.trial.elapsed / state.trial.duration) * 100);
    elements.trialProgressBar.style.width = `${trialPct}%`;

    if (state.trial.target) {
      const phiStr = state.trial.target.phi !== undefined ? `, φ=${(state.trial.target.phi * 180 / Math.PI).toFixed(0)}°` : '';
      elements.trialTargetCoord.textContent = `(${state.trial.target.x.toFixed(2)}, ${state.trial.target.y.toFixed(2)}${phiStr})`;
    } else {
      elements.trialTargetCoord.textContent = '—';
    }

    elements.trialError.textContent = state.trial.error !== null && state.trial.error !== undefined
      ? `${state.trial.error.toFixed(4)} m`
      : '—';

    elements.trialActionState.textContent = state.trial.actionStatus;
    elements.trialActionState.className = `badge ${state.trial.actionStatus}`;

    // Floating Banner
    if (state.trial.running) {
      elements.trialFloatingBanner.classList.remove('hidden');
      elements.floatingBannerTitle.textContent = `IK TRIAL RUNNING [${state.trial.targetsReached} REACHED]`;
      const rem = Math.max(0, state.trial.duration - state.trial.elapsed);
      elements.floatingBannerSubtitle.textContent = `Remaining: ${rem.toFixed(1)}s | Current Error: ${state.trial.error ? state.trial.error.toFixed(3) : '—'} m`;
      elements.floatingBannerGauge.style.width = `${trialPct}%`;
    } else {
      elements.trialFloatingBanner.classList.add('hidden');
    }
  });
}

async function fetchInitialParams() {
  try {
    const res = await ros.callService('/arm_sim/set_params', {});
    if (res.values) {
      if (res.values.gravity !== undefined) state.params.gravity = res.values.gravity;
      if (res.values.lengths) state.params.lengths = res.values.lengths;
      if (res.values.masses) state.params.masses = res.values.masses;
      if (res.values.lengths && res.values.lengths.length !== state.linkCount) {
        setLinkCount(res.values.lengths.length);
      }
      updateParamUI();
    }
  } catch (e) {
    console.warn('Initial params query:', e.message);
  }

  try {
    const res = await ros.callService('/arm_sim/set_integrator', {});
    if (res.values) {
      if (res.values.method) {
        state.integrator = res.values.method;
        elements.hudIntegrator.textContent = state.integrator.toUpperCase();
        const radio = document.querySelector(`input[name="integratorMethod"][value="${state.integrator}"]`);
        if (radio) radio.checked = true;
      }
      if (res.values.timestep) {
        state.timestep = res.values.timestep;
        elements.sliderTimestep.value = state.timestep;
        elements.valTimestep.textContent = `${state.timestep.toFixed(3)} s`;
        elements.hudDt.textContent = `${(state.timestep * 1000).toFixed(1)} ms`;
      }
    }
  } catch (e) {
    console.warn('Initial integrator query:', e.message);
  }
}

// ================= UI SYNCHRONIZATION =================
function updateConnectionStatus(connected) {
  const dot = elements.connStatus.querySelector('.status-dot');
  const label = elements.connStatus.querySelector('.status-label');
  if (connected) {
    dot.className = 'status-dot connected';
    label.textContent = `Online (:9095)`;
  } else {
    dot.className = 'status-dot disconnected';
    label.textContent = 'Disconnected';
  }
}

function setLinkCount(n) {
  if (n !== 2 && n !== 3) return;
  state.linkCount = n;
  
  if (n === 2) {
    elements.btnMode2Link.classList.add('active');
    elements.btnMode3Link.classList.remove('active');
    elements.hudPhiContainer.classList.add('hidden');
    elements.ikPhiRow.classList.add('hidden');
    elements.actionPhiRow.classList.add('hidden');
  } else {
    elements.btnMode3Link.classList.add('active');
    elements.btnMode2Link.classList.remove('active');
    elements.hudPhiContainer.classList.remove('hidden');
    elements.ikPhiRow.classList.remove('hidden');
    elements.actionPhiRow.classList.remove('hidden');
  }

  // Ensure parameter arrays match link count
  while (state.params.lengths.length < n) state.params.lengths.push(1.0);
  while (state.params.lengths.length > n) state.params.lengths.pop();
  while (state.params.masses.length < n) state.params.masses.push(1.0);
  while (state.params.masses.length > n) state.params.masses.pop();
  while (state.kp.length < n) state.kp.push(40.0);
  while (state.kp.length > n) state.kp.pop();
  while (state.ki.length < n) state.ki.push(8.0);
  while (state.ki.length > n) state.ki.pop();
  while (state.kd.length < n) state.kd.push(10.0);
  while (state.kd.length > n) state.kd.pop();
  while (state.setpointPos.length < n) state.setpointPos.push(0.0);
  while (state.setpointPos.length > n) state.setpointPos.pop();

  updateParamUI();
  updatePidUI();
  updateGainsUI('kp');
}

function updateParamUI() {
  elements.sliderGravity.value = state.params.gravity;
  elements.valGravity.textContent = `${state.params.gravity.toFixed(2)} m/s²`;

  // Length sliders
  elements.linkLengthSliders.innerHTML = '';
  for (let i = 0; i < state.linkCount; i++) {
    const val = state.params.lengths[i] || 1.0;
    const row = document.createElement('div');
    row.className = 'slider-label-row mt-1';
    row.innerHTML = `
      <label>Link ${i + 1} (l${i + 1})</label>
      <span class="slider-val" id="valLen${i}">${val.toFixed(2)} m</span>
    `;
    const slider = document.createElement('input');
    slider.type = 'range';
    slider.min = '0.3';
    slider.max = '2.5';
    slider.step = '0.05';
    slider.value = val;
    slider.oninput = (e) => {
      const v = parseFloat(e.target.value);
      state.params.lengths[i] = v;
      document.getElementById(`valLen${i}`).textContent = `${v.toFixed(2)} m`;
    };
    elements.linkLengthSliders.appendChild(row);
    elements.linkLengthSliders.appendChild(slider);
  }

  // Mass sliders
  elements.linkMassSliders.innerHTML = '';
  for (let i = 0; i < state.linkCount; i++) {
    const val = state.params.masses[i] || 1.0;
    const row = document.createElement('div');
    row.className = 'slider-label-row mt-1';
    row.innerHTML = `
      <label>Mass ${i + 1} (m${i + 1})</label>
      <span class="slider-val" id="valMass${i}">${val.toFixed(2)} kg</span>
    `;
    const slider = document.createElement('input');
    slider.type = 'range';
    slider.min = '0.2';
    slider.max = '10.0';
    slider.step = '0.1';
    slider.value = val;
    slider.oninput = (e) => {
      const v = parseFloat(e.target.value);
      state.params.masses[i] = v;
      document.getElementById(`valMass${i}`).textContent = `${v.toFixed(2)} kg`;
    };
    elements.linkMassSliders.appendChild(row);
    elements.linkMassSliders.appendChild(slider);
  }
}

function updatePidUI() {
  elements.chkPidEnable.checked = state.pidEnabled;
  elements.pidStatusBanner.textContent = state.pidEnabled
    ? 'PID Controller is ACTIVE and commanding joint efforts'
    : 'PID Controller is DISABLED (Free Swing under gravity)';
  elements.pidStatusBanner.className = state.pidEnabled ? 'status-banner enabled' : 'status-banner';

  elements.jointSetpointSliders.innerHTML = '';
  for (let i = 0; i < state.linkCount; i++) {
    const deg = ((state.setpointPos[i] || 0) * 180 / Math.PI).toFixed(1);
    const row = document.createElement('div');
    row.className = 'slider-label-row mt-1';
    row.innerHTML = `
      <label>Joint ${i + 1} Target</label>
      <span class="slider-val" id="valSet${i}">${deg}° (${(state.setpointPos[i] || 0).toFixed(2)} rad)</span>
    `;
    const slider = document.createElement('input');
    slider.type = 'range';
    slider.min = '-180';
    slider.max = '180';
    slider.step = '1';
    slider.value = deg;
    slider.oninput = (e) => {
      const d = parseFloat(e.target.value);
      const rad = d * Math.PI / 180.0;
      state.setpointPos[i] = rad;
      document.getElementById(`valSet${i}`).textContent = `${d.toFixed(1)}° (${rad.toFixed(2)} rad)`;
    };
    elements.jointSetpointSliders.appendChild(row);
    elements.jointSetpointSliders.appendChild(slider);
  }
}

let activeGainType = 'kp';
function updateGainsUI(gainType) {
  activeGainType = gainType;
  const list = gainType === 'kp' ? state.kp : (gainType === 'ki' ? state.ki : state.kd);
  const maxVal = gainType === 'kp' ? 150 : 30;
  elements.gainsContainer.innerHTML = '';
  for (let i = 0; i < state.linkCount; i++) {
    const row = document.createElement('div');
    row.className = 'slider-label-row mt-1';
    row.innerHTML = `
      <label>Joint ${i + 1} ${gainType.toUpperCase()}</label>
      <span class="slider-val" id="valGain${i}">${list[i].toFixed(1)}</span>
    `;
    const slider = document.createElement('input');
    slider.type = 'range';
    slider.min = '0';
    slider.max = `${maxVal}`;
    slider.step = '0.5';
    slider.value = list[i];
    slider.oninput = (e) => {
      const v = parseFloat(e.target.value);
      list[i] = v;
      document.getElementById(`valGain${i}`).textContent = v.toFixed(1);
    };
    elements.gainsContainer.appendChild(row);
    elements.gainsContainer.appendChild(slider);
  }
}

// ================= USER INTERACTION & CONTROLS =================
function setupEventHandlers() {
  // Play / Pause Toggle
  elements.btnPlayPause.addEventListener('click', async () => {
    state.paused = !state.paused;
    elements.playIcon.textContent = state.paused ? '▶ Resume' : '⏸ Pause';
    elements.btnPlayPause.className = state.paused ? 'btn btn-success' : 'btn btn-primary';
    try {
      await ros.callService('/arm_sim/pause', { data: state.paused });
    } catch (e) {
      console.error(e);
    }
  });

  // Step physics
  elements.btnStep.addEventListener('click', async () => {
    if (!state.paused) {
      state.paused = true;
      elements.playIcon.textContent = '▶ Resume';
      elements.btnPlayPause.className = 'btn btn-success';
      await ros.callService('/arm_sim/pause', { data: true });
    }
    // Briefly unpause or let server step
    await ros.callService('/arm_sim/pause', { data: false });
    setTimeout(async () => {
      await ros.callService('/arm_sim/pause', { data: true });
    }, 15);
  });

  // Reset simulation
  elements.btnReset.addEventListener('click', async () => {
    try {
      await ros.callService('/arm_sim/reset', {});
      state.trail = [];
    } catch (e) {
      console.error(e);
    }
  });

  // Trail buttons
  elements.btnToggleTrail.addEventListener('click', () => {
    state.trailEnabled = !state.trailEnabled;
    elements.btnToggleTrail.classList.toggle('active', state.trailEnabled);
  });

  elements.btnClearTrail.addEventListener('click', () => {
    state.trail = [];
  });

  // Mode Switcher (2-Link vs 3-Link)
  elements.btnMode2Link.addEventListener('click', async () => {
    if (state.linkCount === 2) return;
    try {
      const res = await fetch('/api/backend/restart?links=2');
      const data = await res.json();
      if (data.success) {
        setLinkCount(2);
        state.trail = [];
      }
    } catch (e) {
      console.error('Mode switch failed:', e);
    }
  });

  elements.btnMode3Link.addEventListener('click', async () => {
    if (state.linkCount === 3) return;
    try {
      const res = await fetch('/api/backend/restart?links=3');
      const data = await res.json();
      if (data.success) {
        setLinkCount(3);
        state.trail = [];
      }
    } catch (e) {
      console.error('Mode switch failed:', e);
    }
  });

  // Integrator selector
  document.querySelectorAll('input[name="integratorMethod"]').forEach((radio) => {
    radio.addEventListener('change', async (e) => {
      const method = e.target.value;
      state.integrator = method;
      elements.hudIntegrator.textContent = method.toUpperCase();
      try {
        await ros.callService('/arm_sim/set_integrator', { method });
      } catch (err) {
        console.error(err);
      }
    });
  });

  // Timestep slider
  elements.sliderTimestep.addEventListener('input', (e) => {
    const dt = parseFloat(e.target.value);
    state.timestep = dt;
    elements.valTimestep.textContent = `${dt.toFixed(3)} s`;
    elements.hudDt.textContent = `${(dt * 1000).toFixed(1)} ms`;
  });
  elements.sliderTimestep.addEventListener('change', async (e) => {
    const dt = parseFloat(e.target.value);
    try {
      await ros.callService('/arm_sim/set_integrator', { timestep: dt });
    } catch (err) {
      console.error(err);
    }
  });

  // Gravity slider
  elements.sliderGravity.addEventListener('input', (e) => {
    const g = parseFloat(e.target.value);
    state.params.gravity = g;
    elements.valGravity.textContent = `${g.toFixed(2)} m/s²`;
  });

  document.querySelectorAll('button[data-gravity]').forEach((btn) => {
    btn.addEventListener('click', async (e) => {
      const g = parseFloat(e.target.getAttribute('data-gravity'));
      state.params.gravity = g;
      elements.sliderGravity.value = g;
      elements.valGravity.textContent = `${g.toFixed(2)} m/s²`;
      await ros.callService('/arm_sim/set_params', { gravity: g });
    });
  });

  // Apply params button
  elements.btnApplyParams.addEventListener('click', async () => {
    try {
      const res = await ros.callService('/arm_sim/set_params', {
        gravity: state.params.gravity,
        lengths: state.params.lengths,
        masses: state.params.masses,
      });
      if (res.values && !res.values.error) {
        elements.btnApplyParams.textContent = '✓ Parameters Applied!';
        setTimeout(() => { elements.btnApplyParams.textContent = 'Apply Physical Parameters'; }, 1500);
      }
    } catch (err) {
      console.error(err);
    }
  });

  // Initial Drop Presets
  elements.btnDropHorizontal.addEventListener('click', async () => {
    await ros.callService('/pid_controller/enable', { data: false });
    state.pidEnabled = false;
    updatePidUI();
    const angles = state.linkCount === 3 ? [0.0, 0.0, 0.0] : [0.0, 0.0];
    await ros.callService('/arm_sim/reset', {});
    // Give small kick
    state.trail = [];
  });

  elements.btnDropUpright.addEventListener('click', async () => {
    // Release from near upright pi/2
    await ros.callService('/pid_controller/enable', { data: false });
    state.pidEnabled = false;
    updatePidUI();
    await ros.callService('/arm_sim/reset', {});
    state.trail = [];
  });

  elements.btnDropHanging.addEventListener('click', async () => {
    await ros.callService('/pid_controller/enable', { data: false });
    state.pidEnabled = false;
    updatePidUI();
    await ros.callService('/arm_sim/reset', {});
    state.trail = [];
  });

  elements.btnKickArm.addEventListener('click', () => {
    // Send a momentary non-zero velocity trajectory
    const vels = state.linkCount === 3 ? [2.5, -3.0, 2.0] : [3.0, -4.0];
    ros.publish('/joint_trajectory', {
      points: [{
        positions: state.q,
        velocities: vels,
      }]
    });
  });

  // PID Controls
  elements.chkPidEnable.addEventListener('change', async (e) => {
    state.pidEnabled = e.target.checked;
    updatePidUI();
    try {
      await ros.callService('/pid_controller/enable', { data: state.pidEnabled });
    } catch (err) {
      console.error(err);
    }
  });

  elements.btnSendTrajectory.addEventListener('click', () => {
    ros.publish('/joint_trajectory', {
      points: [{
        positions: state.setpointPos,
        velocities: new Array(state.linkCount).fill(0.0),
      }]
    });
  });

  elements.btnZeroSetpoints.addEventListener('click', () => {
    state.setpointPos = new Array(state.linkCount).fill(0.0);
    updatePidUI();
    elements.btnSendTrajectory.click();
  });

  // Gains Sub-tabs
  document.querySelectorAll('.tab-sub').forEach((btn) => {
    btn.addEventListener('click', (e) => {
      document.querySelectorAll('.tab-sub').forEach(b => b.classList.remove('active'));
      e.target.classList.add('active');
      updateGainsUI(e.target.getAttribute('data-sub'));
    });
  });

  elements.btnApplyGains.addEventListener('click', async () => {
    try {
      await ros.callService('/pid_controller/set_gains', {
        kp: state.kp,
        ki: state.ki,
        kd: state.kd,
      });
      elements.btnApplyGains.textContent = '✓ Gains Applied';
      setTimeout(() => { elements.btnApplyGains.textContent = 'Apply Gains'; }, 1500);
    } catch (err) {
      console.error(err);
    }
  });

  elements.btnPresetStiff.addEventListener('click', () => {
    state.kp = state.linkCount === 3 ? [80.0, 80.0, 60.0] : [80.0, 80.0];
    state.ki = state.linkCount === 3 ? [15.0, 15.0, 10.0] : [15.0, 15.0];
    state.kd = state.linkCount === 3 ? [20.0, 20.0, 15.0] : [20.0, 20.0];
    updateGainsUI(activeGainType);
    elements.btnApplyGains.click();
  });

  elements.btnPresetSoft.addEventListener('click', () => {
    state.kp = state.linkCount === 3 ? [20.0, 20.0, 15.0] : [20.0, 20.0];
    state.ki = state.linkCount === 3 ? [2.0, 2.0, 1.0] : [2.0, 2.0];
    state.kd = state.linkCount === 3 ? [5.0, 5.0, 4.0] : [5.0, 5.0];
    updateGainsUI(activeGainType);
    elements.btnApplyGains.click();
  });

  // IK Tab
  elements.ikTargetPhiSlider.addEventListener('input', (e) => {
    const deg = parseFloat(e.target.value);
    elements.ikTargetPhiVal.textContent = `${deg.toFixed(1)}°`;
    state.ikTarget.phi = deg * Math.PI / 180.0;
  });

  elements.btnSolveIk.addEventListener('click', async () => {
    const x = parseFloat(elements.ikTargetX.value);
    const y = parseFloat(elements.ikTargetY.value);
    const phi = state.linkCount === 3 ? state.ikTarget.phi : undefined;
    
    state.ikTarget.x = x;
    state.ikTarget.y = y;

    try {
      const args = { x, y };
      if (phi !== undefined) args.phi = phi;
      const res = await ros.callService('/ik/solve', args);
      if (res.values && res.values.positions) {
        state.ikSolution = res.values.positions;
        const degs = state.ikSolution.map(r => (r * 180 / Math.PI).toFixed(1) + '°');
        elements.ikResultCard.innerHTML = `
          <div class="card-row">
            <span class="label">Status:</span>
            <span class="badge reached">Solved Successfully</span>
          </div>
          <div class="card-row">
            <span class="label">Joint Angles:</span>
            <span class="val font-mono">[${degs.join(', ')}]</span>
          </div>
          <div class="card-row">
            <span class="label">Radians:</span>
            <span class="val font-mono">[${state.ikSolution.map(r => r.toFixed(3)).join(', ')}]</span>
          </div>
        `;
        elements.btnApplyIkToPid.classList.remove('hidden');
      } else {
        state.ikSolution = null;
        elements.ikResultCard.innerHTML = `
          <div class="card-row">
            <span class="label">Status:</span>
            <span class="badge active" style="color:var(--accent-rose);border-color:var(--accent-rose);">Unreachable</span>
          </div>
          <div class="card-row mt-1">
            <span class="val" style="color:var(--accent-rose);font-size:11px;">${res.values?.error || 'Target outside reach'}</span>
          </div>
        `;
        elements.btnApplyIkToPid.classList.add('hidden');
      }
    } catch (err) {
      console.error(err);
    }
  });

  elements.btnApplyIkToPid.addEventListener('click', async () => {
    if (!state.ikSolution) return;
    if (!state.pidEnabled) {
      state.pidEnabled = true;
      elements.chkPidEnable.checked = true;
      updatePidUI();
      await ros.callService('/pid_controller/enable', { data: true });
    }
    state.setpointPos = [...state.ikSolution];
    updatePidUI();
    ros.publish('/joint_trajectory', {
      points: [{
        positions: state.ikSolution,
        velocities: new Array(state.linkCount).fill(0.0),
      }]
    });
  });

  // Action Tab
  elements.btnSendGoal.addEventListener('click', async () => {
    const x = parseFloat(elements.actionTargetX.value);
    const y = parseFloat(elements.actionTargetY.value);
    const phi = state.linkCount === 3 ? (parseFloat(elements.actionTargetPhi.value) * Math.PI / 180.0) : undefined;
    const epsilon = parseFloat(elements.actionEpsilon.value);
    const success_hold = parseFloat(elements.actionSuccessHold.value);

    // Automatically enable PID if not already active
    if (!state.pidEnabled) {
      state.pidEnabled = true;
      elements.chkPidEnable.checked = true;
      updatePidUI();
      await ros.callService('/pid_controller/enable', { data: true });
    }

    try {
      const args = { x, y, epsilon, success_hold };
      if (phi !== undefined) args.phi = phi;
      await ros.callService('/ik_action/send_goal', args);
    } catch (err) {
      console.error(err);
    }
  });

  elements.btnCancelGoal.addEventListener('click', async () => {
    try {
      await ros.callService('/ik_action/cancel_goal', {});
    } catch (err) {
      console.error(err);
    }
  });

  // Trial Tab
  elements.btnStartTrial.addEventListener('click', async () => {
    const duration = parseFloat(elements.trialDuration.value);
    const epsilon = parseFloat(elements.trialEpsilon.value);
    const success_hold = parseFloat(elements.trialHold.value);

    if (!state.pidEnabled) {
      state.pidEnabled = true;
      elements.chkPidEnable.checked = true;
      updatePidUI();
      await ros.callService('/pid_controller/enable', { data: true });
    }

    try {
      await ros.callService('/ik_trial/start', { duration, epsilon, success_hold });
    } catch (err) {
      console.error(err);
    }
  });

  elements.btnSkipTrial.addEventListener('click', async () => {
    try {
      await ros.callService('/ik_trial/skip', {});
    } catch (err) {
      console.error(err);
    }
  });

  elements.btnStopTrial.addEventListener('click', async () => {
    try {
      await ros.callService('/ik_trial/stop', {});
    } catch (err) {
      console.error(err);
    }
  });

  // ODE Playground Tab
  elements.btnOdeStep.addEventListener('click', async () => {
    const expr = elements.odeExpression.value;
    const t0 = parseFloat(elements.odeT0.value);
    const x0 = parseFloat(elements.odeX0.value);
    const v0 = parseFloat(elements.odeV0.value);
    const dt = parseFloat(elements.odeDt.value);
    const method = elements.odeMethod.value;

    try {
      const res = await ros.callService('/arm_sim/integration_step', {
        expression: expr,
        t0, x0, v0, dt, method
      });

      if (res.values && res.values.x !== undefined) {
        elements.odeResultCard.innerHTML = `
          <div class="card-row"><span class="label">t:</span><span class="val font-mono">${res.values.t.toFixed(4)}</span></div>
          <div class="card-row"><span class="label">x:</span><span class="val font-mono highlight">${res.values.x.toFixed(6)}</span></div>
          <div class="card-row"><span class="label">v:</span><span class="val font-mono highlight">${res.values.v.toFixed(6)}</span></div>
        `;
        // Advance inputs for next step
        elements.odeT0.value = res.values.t.toFixed(4);
        elements.odeX0.value = res.values.x.toFixed(4);
        elements.odeV0.value = res.values.v.toFixed(4);
      } else {
        elements.odeResultCard.innerHTML = `<span style="color:var(--accent-rose);">${res.values?.error || 'Integration error'}</span>`;
      }
    } catch (err) {
      elements.odeResultCard.innerHTML = `<span style="color:var(--accent-rose);">${err.message}</span>`;
    }
  });

  elements.btnOdeSimulate.addEventListener('click', async () => {
    const expr = elements.odeExpression.value;
    let t = parseFloat(elements.odeT0.value);
    let x = parseFloat(elements.odeX0.value);
    let v = parseFloat(elements.odeV0.value);
    const dt = parseFloat(elements.odeDt.value);
    const method = elements.odeMethod.value;

    elements.odeResultCard.innerHTML = `Simulating 500 steps...`;

    for (let step = 0; step < 500; step++) {
      try {
        const res = await ros.callService('/arm_sim/integration_step', {
          expression: expr, t0: t, x0: x, v0: v, dt, method
        });
        if (!res.values || res.values.x === undefined) break;
        t = res.values.t;
        x = res.values.x;
        v = res.values.v;
      } catch (e) {
        break;
      }
    }

    elements.odeT0.value = t.toFixed(4);
    elements.odeX0.value = x.toFixed(4);
    elements.odeV0.value = v.toFixed(4);
    elements.odeResultCard.innerHTML = `
      <div>500 Steps Completed:</div>
      <div class="card-row"><span class="label">Final t:</span><span class="val font-mono">${t.toFixed(4)}</span></div>
      <div class="card-row"><span class="label">Final x:</span><span class="val font-mono highlight">${x.toFixed(4)}</span></div>
      <div class="card-row"><span class="label">Final v:</span><span class="val font-mono highlight">${v.toFixed(4)}</span></div>
    `;
  });

  // Deck Tabs Navigation
  document.querySelectorAll('.deck-tab').forEach((tab) => {
    tab.addEventListener('click', (e) => {
      document.querySelectorAll('.deck-tab').forEach(t => t.classList.remove('active'));
      document.querySelectorAll('.tab-pane').forEach(p => p.classList.remove('active'));
      e.target.classList.add('active');
      const paneId = `pane-${e.target.getAttribute('data-tab')}`;
      document.getElementById(paneId).classList.add('active');
    });
  });

  // Chart Tabs Navigation
  document.querySelectorAll('.chart-tab').forEach((tab) => {
    tab.addEventListener('click', (e) => {
      document.querySelectorAll('.chart-tab').forEach(t => t.classList.remove('active'));
      e.target.classList.add('active');
      state.activeChartTab = e.target.getAttribute('data-chart');
      updateChartLegend();
    });
  });

  // Canvas Mouse Controls (Zoom, Pan, Drag Target)
  const canvas = elements.armCanvas;
  
  canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    const zoomFactor = e.deltaY < 0 ? 1.1 : 0.9;
    state.view.scale = Math.max(50, Math.min(600, state.view.scale * zoomFactor));
  }, { passive: false });

  elements.btnZoomIn.addEventListener('click', () => {
    state.view.scale = Math.min(600, state.view.scale * 1.2);
  });

  elements.btnZoomOut.addEventListener('click', () => {
    state.view.scale = Math.max(50, state.view.scale / 1.2);
  });

  elements.btnResetView.addEventListener('click', () => {
    resetCanvasView();
  });

  canvas.addEventListener('mousedown', (e) => {
    if (e.button === 0) { // Left click: Set target / drag
      state.view.isDraggingTarget = true;
      updateTargetFromPointer(e.clientX, e.clientY);
    } else if (e.button === 1 || e.button === 2) { // Middle or Right click: Pan
      state.view.isPanning = true;
      state.view.panStartX = e.clientX - state.view.offsetX;
      state.view.panStartY = e.clientY - state.view.offsetY;
    }
  });

  window.addEventListener('mousemove', (e) => {
    if (state.view.isDraggingTarget) {
      updateTargetFromPointer(e.clientX, e.clientY);
    } else if (state.view.isPanning) {
      state.view.offsetX = e.clientX - state.view.panStartX;
      state.view.offsetY = e.clientY - state.view.panStartY;
    }
  });

  window.addEventListener('mouseup', () => {
    if (state.view.isDraggingTarget) {
      state.view.isDraggingTarget = false;
      // Auto solve IK or send goal if in action tab
      const activeTab = document.querySelector('.deck-tab.active')?.getAttribute('data-tab');
      if (activeTab === 'action') {
        elements.btnSendGoal.click();
      } else {
        elements.btnSolveIk.click();
      }
    }
    state.view.isPanning = false;
  });

  canvas.addEventListener('contextmenu', (e) => e.preventDefault());

  // Global Keyboard Shortcuts
  window.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
    if (e.code === 'Space') {
      e.preventDefault();
      elements.btnPlayPause.click();
    } else if (e.code === 'KeyR') {
      elements.btnReset.click();
    } else if (e.code === 'KeyT') {
      elements.btnToggleTrail.click();
    } else if (e.code === 'KeyC') {
      elements.btnClearTrail.click();
    }
  });
}

function updateTargetFromPointer(clientX, clientY) {
  const rect = elements.armCanvas.getBoundingClientRect();
  const px = clientX - rect.left;
  const py = clientY - rect.top;

  // Convert to world meters:
  // Center is (width/2 + offsetX, height/2 + offsetY)
  // World +X is right, World +Y is UP
  const centerX = elements.armCanvas.width / (2 * window.devicePixelRatio) + state.view.offsetX;
  const centerY = elements.armCanvas.height / (2 * window.devicePixelRatio) + state.view.offsetY;

  const worldX = (px - centerX) / state.view.scale;
  const worldY = -(py - centerY) / state.view.scale;

  state.ikTarget.x = worldX;
  state.ikTarget.y = worldY;

  elements.ikTargetX.value = worldX.toFixed(2);
  elements.ikTargetY.value = worldY.toFixed(2);
  elements.actionTargetX.value = worldX.toFixed(2);
  elements.actionTargetY.value = worldY.toFixed(2);

  // Compute live client-side ghost arm preview
  if (state.linkCount === 2) {
    state.ikSolution = solve2LinkClient(state.params.lengths[0], state.params.lengths[1], worldX, worldY);
  } else {
    state.ikSolution = solve3LinkClient(state.params.lengths[0], state.params.lengths[1], state.params.lengths[2], worldX, worldY, state.ikTarget.phi);
  }
}

function resetCanvasView() {
  state.view.offsetX = 0;
  state.view.offsetY = 0;
  const totalLength = state.params.lengths.reduce((a, b) => a + b, 0);
  const minDim = Math.min(elements.armCanvas.width, elements.armCanvas.height) / window.devicePixelRatio;
  state.view.scale = (minDim * 0.38) / (totalLength || 2.0);
}

// ================= CANVAS RENDERING ENGINE =================
function resizeCanvas() {
  const dpr = window.devicePixelRatio || 1;
  const rect = elements.canvasWrapper.getBoundingClientRect();
  if (elements.armCanvas.width !== rect.width * dpr || elements.armCanvas.height !== rect.height * dpr) {
    elements.armCanvas.width = rect.width * dpr;
    elements.armCanvas.height = rect.height * dpr;
  }

  const chartRect = elements.telemetryChart.parentElement.getBoundingClientRect();
  if (elements.telemetryChart.width !== chartRect.width * dpr || elements.telemetryChart.height !== chartRect.height * dpr) {
    elements.telemetryChart.width = chartRect.width * dpr;
    elements.telemetryChart.height = chartRect.height * dpr;
  }
}

function render() {
  resizeCanvas();
  const dpr = window.devicePixelRatio || 1;
  const w = elements.armCanvas.width / dpr;
  const h = elements.armCanvas.height / dpr;

  armCtx.save();
  armCtx.scale(dpr, dpr);
  armCtx.clearRect(0, 0, w, h);

  // Center coordinate origin
  const originX = w / 2 + state.view.offsetX;
  const originY = h / 2 + state.view.offsetY;

  // 1. Draw World Coordinate Grid
  drawGrid(armCtx, w, h, originX, originY);

  // 2. Draw Reachable Workspace Circles
  drawWorkspace(armCtx, originX, originY);

  // 3. Draw End-Effector Motion Trail
  if (state.trailEnabled && state.trail.length > 1) {
    drawTrail(armCtx, originX, originY);
  }

  // 4. Draw Ghost Arm (IK solution / candidate target preview)
  if (state.ikSolution) {
    drawArm(armCtx, originX, originY, state.ikSolution, state.params.lengths, true);
  }

  // 5. Draw Primary Physical Arm
  drawArm(armCtx, originX, originY, state.q, state.params.lengths, false);

  // 6. Draw Targets & Tolerance Rings (IK Action / Trial)
  drawTargets(armCtx, originX, originY);

  armCtx.restore();

  // Render Telemetry Chart
  renderTelemetryChart();

  // FPS calculation
  const now = performance.now();
  state.frameCount++;
  if (now - state.lastFpsUpdate >= 500) {
    state.fps = Math.round((state.frameCount * 1000) / (now - state.lastFpsUpdate));
    elements.hudFps.textContent = state.fps;
    state.frameCount = 0;
    state.lastFpsUpdate = now;
  }

  requestAnimationFrame(render);
}

function drawGrid(ctx, w, h, ox, oy) {
  const scale = state.view.scale;
  const gridMeters = scale < 100 ? 1.0 : (scale < 250 ? 0.5 : 0.25);
  const gridPixels = gridMeters * scale;

  ctx.strokeStyle = 'rgba(255, 255, 255, 0.04)';
  ctx.lineWidth = 1;

  // Vertical lines
  const startX = (ox % gridPixels) - gridPixels;
  for (let x = startX; x <= w + gridPixels; x += gridPixels) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, h);
    ctx.stroke();
  }

  // Horizontal lines
  const startY = (oy % gridPixels) - gridPixels;
  for (let y = startY; y <= h + gridPixels; y += gridPixels) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }

  // Axes (X and Y)
  ctx.strokeStyle = 'rgba(0, 240, 255, 0.2)';
  ctx.lineWidth = 1.5;
  
  // X axis
  ctx.beginPath();
  ctx.moveTo(0, oy);
  ctx.lineTo(w, oy);
  ctx.stroke();

  // Y axis
  ctx.beginPath();
  ctx.moveTo(ox, 0);
  ctx.lineTo(ox, h);
  ctx.stroke();

  // Meter distance annotations
  ctx.fillStyle = 'rgba(255, 255, 255, 0.25)';
  ctx.font = '9px Fira Code';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';

  for (let x = startX; x <= w + gridPixels; x += gridPixels) {
    const meterVal = ((x - ox) / scale).toFixed(1);
    if (Math.abs(x - ox) > 10) {
      ctx.fillText(`${meterVal}m`, x, oy + 4);
    }
  }
}

function drawWorkspace(ctx, ox, oy) {
  const scale = state.view.scale;
  const totalLength = state.params.lengths.reduce((a, b) => a + b, 0);
  
  // Outer reachable circle
  ctx.strokeStyle = 'rgba(0, 240, 255, 0.12)';
  ctx.lineWidth = 1;
  ctx.setLineDash([4, 4]);
  ctx.beginPath();
  ctx.arc(ox, oy, totalLength * scale, 0, Math.PI * 2);
  ctx.stroke();

  // Inner boundary for 2-link
  if (state.linkCount === 2) {
    const minReach = Math.abs(state.params.lengths[0] - state.params.lengths[1]);
    if (minReach > 0.01) {
      ctx.beginPath();
      ctx.arc(ox, oy, minReach * scale, 0, Math.PI * 2);
      ctx.stroke();
    }
  }
  ctx.setLineDash([]);
}

function drawTrail(ctx, ox, oy) {
  const scale = state.view.scale;
  const trail = state.trail;
  const count = trail.length;

  ctx.lineWidth = 2;
  for (let i = 1; i < count; i++) {
    const alpha = (i / count) * 0.7;
    ctx.strokeStyle = `rgba(0, 240, 255, ${alpha})`;
    const p1x = ox + trail[i - 1].x * scale;
    const p1y = oy - trail[i - 1].y * scale;
    const p2x = ox + trail[i].x * scale;
    const p2y = oy - trail[i].y * scale;
    ctx.beginPath();
    ctx.moveTo(p1x, p1y);
    ctx.lineTo(p2x, p2y);
    ctx.stroke();
  }
}

function drawArm(ctx, ox, oy, q, lengths, isGhost = false) {
  const scale = state.view.scale;
  const { joints, ee } = forwardKinematics(q, lengths);
  const n = lengths.length;

  const linkColors = [
    '#00d2ff', // Link 1: Cyan
    '#ff9900', // Link 2: Amber
    '#10b981', // Link 3: Emerald
  ];

  // Base Pivot Mount (fixed to ground)
  if (!isGhost) {
    ctx.fillStyle = '#1e2633';
    ctx.strokeStyle = '#3d4b61';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(ox, oy, 14, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    // Base hatching
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(ox - 18, oy + 12);
    ctx.lineTo(ox + 18, oy + 12);
    ctx.moveTo(ox - 12, oy + 12);
    ctx.lineTo(ox - 18, oy + 20);
    ctx.moveTo(ox, oy + 12);
    ctx.lineTo(ox - 6, oy + 20);
    ctx.moveTo(ox + 12, oy + 12);
    ctx.lineTo(ox + 6, oy + 20);
    ctx.stroke();
  }

  // Draw Links
  for (let i = 0; i < n; i++) {
    const j1 = joints[i];
    const j2 = joints[i + 1];

    const p1x = ox + j1.x * scale;
    const p1y = oy - j1.y * scale;
    const p2x = ox + j2.x * scale;
    const p2y = oy - j2.y * scale;

    const color = linkColors[i % linkColors.length];

    if (isGhost) {
      // Ghost dashed line
      ctx.strokeStyle = 'rgba(0, 240, 255, 0.4)';
      ctx.lineWidth = 4;
      ctx.setLineDash([6, 6]);
      ctx.beginPath();
      ctx.moveTo(p1x, p1y);
      ctx.lineTo(p2x, p2y);
      ctx.stroke();
      ctx.setLineDash([]);
    } else {
      // Physical Rod Link
      ctx.save();
      const grad = ctx.createLinearGradient(p1x, p1y, p2x, p2y);
      grad.addColorStop(0, color);
      grad.addColorStop(1, '#ffffff');

      ctx.strokeStyle = color;
      ctx.lineWidth = 9;
      ctx.lineCap = 'round';
      ctx.shadowColor = color;
      ctx.shadowBlur = 10;
      ctx.beginPath();
      ctx.moveTo(p1x, p1y);
      ctx.lineTo(p2x, p2y);
      ctx.stroke();
      ctx.restore();

      // Inner shiny rod core
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 2.5;
      ctx.lineCap = 'round';
      ctx.beginPath();
      ctx.moveTo(p1x, p1y);
      ctx.lineTo(p2x, p2y);
      ctx.stroke();

      // Center of Mass indicator at midpoint
      const comX = (p1x + p2x) / 2;
      const comY = (p1y + p2y) / 2;
      ctx.fillStyle = '#ff3b69';
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.arc(comX, comY, 3.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      // Joint circle
      ctx.fillStyle = '#10141a';
      ctx.strokeStyle = color;
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.arc(p1x, p1y, 7, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }
  }

  // End-Effector Tip
  const eeX = ox + ee.x * scale;
  const eeY = oy - ee.y * scale;

  if (isGhost) {
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.6)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(eeX, eeY, 8, 0, Math.PI * 2);
    ctx.stroke();
  } else {
    // Glowing Cyan Flange / Gripper
    ctx.save();
    ctx.fillStyle = '#00f0ff';
    ctx.shadowColor = '#00f0ff';
    ctx.shadowBlur = 15;
    ctx.beginPath();
    ctx.arc(eeX, eeY, 7, 0, Math.PI * 2);
    ctx.fill();

    // Orientation arrow for 3-link
    if (state.linkCount === 3) {
      const arrowLen = 22;
      const ax = eeX + arrowLen * Math.cos(ee.phi);
      const ay = eeY - arrowLen * Math.sin(ee.phi);
      ctx.strokeStyle = '#00ffaa';
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.moveTo(eeX, eeY);
      ctx.lineTo(ax, ay);
      ctx.stroke();
    }
    ctx.restore();
  }
}

function drawTargets(ctx, ox, oy) {
  const scale = state.view.scale;

  // Active IK Action Goal
  if (state.activeGoal && state.activeGoal.target) {
    const tg = state.activeGoal.target;
    const tx = ox + tg.x * scale;
    const ty = oy - tg.y * scale;
    const eps = (elements.actionEpsilon.value ? parseFloat(elements.actionEpsilon.value) : 0.05) * scale;

    // Tolerance circle
    ctx.save();
    ctx.strokeStyle = 'rgba(0, 255, 170, 0.4)';
    ctx.fillStyle = 'rgba(0, 255, 170, 0.08)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(tx, ty, eps, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    // Target crosshair
    ctx.strokeStyle = '#00ffaa';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(tx - 9, ty); ctx.lineTo(tx + 9, ty);
    ctx.moveTo(tx, ty - 9); ctx.lineTo(tx, ty + 9);
    ctx.stroke();
    ctx.restore();
  }

  // Active Trial Target
  if (state.trial.running && state.trial.target) {
    const tg = state.trial.target;
    const tx = ox + tg.x * scale;
    const ty = oy - tg.y * scale;
    const eps = (state.trial.epsilon || 0.05) * scale;

    ctx.save();
    // Pulsing outer ring
    const pulse = 1.0 + 0.15 * Math.sin(performance.now() * 0.008);
    ctx.strokeStyle = '#ffb800';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(tx, ty, eps * pulse, 0, Math.PI * 2);
    ctx.stroke();

    ctx.fillStyle = 'rgba(255, 184, 0, 0.15)';
    ctx.beginPath();
    ctx.arc(tx, ty, eps, 0, Math.PI * 2);
    ctx.fill();

    // Crosshair
    ctx.strokeStyle = '#ffb800';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(tx - 10, ty); ctx.lineTo(tx + 10, ty);
    ctx.moveTo(tx, ty - 10); ctx.lineTo(tx, ty + 10);
    ctx.stroke();
    ctx.restore();
  }

  // Interactive Target Indicator (when dragging / selecting)
  if (!state.trial.running && !state.activeGoal && state.ikTarget) {
    const tx = ox + state.ikTarget.x * scale;
    const ty = oy - state.ikTarget.y * scale;

    ctx.save();
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.6)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(tx, ty, 8, 0, Math.PI * 2);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(tx - 6, ty); ctx.lineTo(tx + 6, ty);
    ctx.moveTo(tx, ty - 6); ctx.lineTo(tx, ty + 6);
    ctx.stroke();
    ctx.restore();
  }
}

// ================= TELEMETRY CHARTS ENGINE =================
function pushTelemetryHistory(time, q, qdot, effort, energy) {
  state.history.time.push(time);
  for (let i = 0; i < 3; i++) {
    state.history.q[i].push(q[i] !== undefined ? q[i] : null);
    state.history.qdot[i].push(qdot[i] !== undefined ? qdot[i] : null);
    state.history.effort[i].push(effort[i] !== undefined ? effort[i] : null);
  }
  state.history.energyT.push(energy.T);
  state.history.energyV.push(energy.V);
  state.history.energyTotal.push(energy.total);

  if (state.history.time.length > state.maxHistoryPoints) {
    state.history.time.shift();
    for (let i = 0; i < 3; i++) {
      state.history.q[i].shift();
      state.history.qdot[i].shift();
      state.history.effort[i].shift();
    }
    state.history.energyT.shift();
    state.history.energyV.shift();
    state.history.energyTotal.shift();
  }
}

function updateChartLegend() {
  const tab = state.activeChartTab;
  let html = '';
  if (tab === 'angles') {
    html = `
      <span class="legend-item"><span class="legend-color" style="background:#00d2ff"></span> q1 (rad)</span>
      <span class="legend-item"><span class="legend-color" style="background:#ff9900"></span> q2 (rad)</span>
      ${state.linkCount === 3 ? '<span class="legend-item"><span class="legend-color" style="background:#10b981"></span> q3 (rad)</span>' : ''}
    `;
  } else if (tab === 'velocities') {
    html = `
      <span class="legend-item"><span class="legend-color" style="background:#00d2ff"></span> q̇1 (rad/s)</span>
      <span class="legend-item"><span class="legend-color" style="background:#ff9900"></span> q̇2 (rad/s)</span>
      ${state.linkCount === 3 ? '<span class="legend-item"><span class="legend-color" style="background:#10b981"></span> q̇3 (rad/s)</span>' : ''}
    `;
  } else if (tab === 'efforts') {
    html = `
      <span class="legend-item"><span class="legend-color" style="background:#00d2ff"></span> τ1 (N·m)</span>
      <span class="legend-item"><span class="legend-color" style="background:#ff9900"></span> τ2 (N·m)</span>
      ${state.linkCount === 3 ? '<span class="legend-item"><span class="legend-color" style="background:#10b981"></span> τ3 (N·m)</span>' : ''}
    `;
  } else if (tab === 'energy') {
    html = `
      <span class="legend-item"><span class="legend-color" style="background:#00ffaa"></span> Kinetic T</span>
      <span class="legend-item"><span class="legend-color" style="background:#ff3b69"></span> Potential V</span>
      <span class="legend-item"><span class="legend-color" style="background:#ffffff"></span> Total E = T + V</span>
    `;
  }
  elements.chartLegend.innerHTML = html;
}

function renderTelemetryChart() {
  const dpr = window.devicePixelRatio || 1;
  const w = elements.telemetryChart.width / dpr;
  const h = elements.telemetryChart.height / dpr;

  chartCtx.save();
  chartCtx.scale(dpr, dpr);
  chartCtx.clearRect(0, 0, w, h);

  const points = state.history.time.length;
  if (points < 2) {
    chartCtx.restore();
    return;
  }

  // Determine series to plot
  let series = [];
  const colors = ['#00d2ff', '#ff9900', '#10b981'];
  if (state.activeChartTab === 'angles') {
    series = state.history.q.slice(0, state.linkCount).map((arr, i) => ({ data: arr, color: colors[i] }));
  } else if (state.activeChartTab === 'velocities') {
    series = state.history.qdot.slice(0, state.linkCount).map((arr, i) => ({ data: arr, color: colors[i] }));
  } else if (state.activeChartTab === 'efforts') {
    series = state.history.effort.slice(0, state.linkCount).map((arr, i) => ({ data: arr, color: colors[i] }));
  } else if (state.activeChartTab === 'energy') {
    series = [
      { data: state.history.energyT, color: '#00ffaa' },
      { data: state.history.energyV, color: '#ff3b69' },
      { data: state.history.energyTotal, color: '#ffffff' },
    ];
  }

  // Find min/max for autoscale
  let minY = Infinity;
  let maxY = -Infinity;
  for (const s of series) {
    for (const val of s.data) {
      if (val !== null && isFinite(val)) {
        if (val < minY) minY = val;
        if (val > maxY) maxY = val;
      }
    }
  }

  if (minY === Infinity || maxY === -Infinity || Math.abs(maxY - minY) < 0.001) {
    minY = -1.0;
    maxY = 1.0;
  }

  // Add 10% vertical padding
  const span = maxY - minY;
  minY -= span * 0.1;
  maxY += span * 0.1;

  // Draw chart grid & Zero line
  chartCtx.strokeStyle = 'rgba(255, 255, 255, 0.06)';
  chartCtx.lineWidth = 1;
  const gridSteps = 4;
  for (let i = 0; i <= gridSteps; i++) {
    const yVal = minY + (span * 1.2 * i) / gridSteps;
    const yPx = h - ((yVal - minY) / (maxY - minY)) * h;
    chartCtx.beginPath();
    chartCtx.moveTo(0, yPx);
    chartCtx.lineTo(w, yPx);
    chartCtx.stroke();
  }

  // Zero Line
  if (minY < 0 && maxY > 0) {
    const zeroY = h - ((0 - minY) / (maxY - minY)) * h;
    chartCtx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
    chartCtx.lineWidth = 1.5;
    chartCtx.beginPath();
    chartCtx.moveTo(0, zeroY);
    chartCtx.lineTo(w, zeroY);
    chartCtx.stroke();
  }

  // Plot lines
  for (const s of series) {
    chartCtx.strokeStyle = s.color;
    chartCtx.lineWidth = 1.5;
    chartCtx.beginPath();
    let started = false;

    for (let i = 0; i < points; i++) {
      const val = s.data[i];
      if (val === null || !isFinite(val)) continue;
      const xPx = (i / (state.maxHistoryPoints - 1)) * w;
      const yPx = h - ((val - minY) / (maxY - minY)) * h;

      if (!started) {
        chartCtx.moveTo(xPx, yPx);
        started = true;
      } else {
        chartCtx.lineTo(xPx, yPx);
      }
    }
    chartCtx.stroke();
  }

  chartCtx.restore();
}

// ================= INITIALIZATION =================
function init() {
  setupEventHandlers();
  setLinkCount(2);
  updateChartLegend();
  resetCanvasView();

  // Connect to Rosbridge WebSocket
  ros.connect();
  subscribeTopics();

  // Start animation loop
  requestAnimationFrame(render);
}

window.addEventListener('DOMContentLoaded', init);
