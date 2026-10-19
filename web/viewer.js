/**
 * Project 3 3D Robot Viewer Frontend
 * ----------------------------------
 * - Three.js WebGL rendering with Z-up coordinates
 * - Real-time column-major matrix updates from /xform_world via SSE
 * - URDF visual geometry parser (box, cylinder, sphere, and mesh proxies)
 * - Stale link object purge on robot reload
 * - Choreography FSM control deck & live telemetry
 * - Canvas video recording via MediaRecorder API
 */

(function () {
  'use strict';

  // --- State ---
  let scene, camera, renderer, controls;
  let robotRootGroup;
  const linkObjects = new Map(); // linkName -> THREE.Group
  let activeRobotDescription = "";
  let activeRobotStatus = null;
  let activeFsmStatus = null;
  let sseSource = null;
  let selectedLinkName = null;
  let animationSpeed = 1.0;
  const _tmpMatrix = new THREE.Matrix4();
  let lastFrameTime = performance.now();

  // MediaRecorder state
  let mediaRecorder = null;
  let recordedChunks = [];
  let isRecording = false;
  let recordingStartTime = 0;
  let recordTimerInterval = null;

  // DOM Elements
  const container = document.getElementById('canvas-container');
  const connDot = document.getElementById('connDot');
  const connText = document.getElementById('connText');
  const robotSelect = document.getElementById('robotSelect');
  const fsmSelect = document.getElementById('fsmSelect');
  const btnStartFsm = document.getElementById('btnStartFsm');
  const btnPauseFsm = document.getElementById('btnPauseFsm');
  const btnResetJoints = document.getElementById('btnResetJoints');
  const btnStepTransition = document.getElementById('btnStepTransition');
  const btnRecord = document.getElementById('btnRecord');
  const recordLabel = document.getElementById('recordLabel');
  const fsmStateBadge = document.getElementById('fsmStateBadge');
  const fsmRunningTag = document.getElementById('fsmRunningTag');
  const fsmTimeInState = document.getElementById('fsmTimeInState');
  const fsmLastTransition = document.getElementById('fsmLastTransition');
  const jointListEl = document.getElementById('jointList');
  const linkCountBadge = document.getElementById('linkCountBadge');
  const selectedLinkNameEl = document.getElementById('selectedLinkName');
  const matrixGridEl = document.getElementById('matrixGrid');
  const posWorldEl = document.getElementById('posWorld');
  const speedSlider = document.getElementById('speedSlider');
  const speedValueLabel = document.getElementById('speedValueLabel');

  // --- Color Palette for Fallback ---
  const DEFAULT_COLORS = [
    0x3b82f6, 0x10b981, 0xf59e0b, 0x8b5cf6,
    0x06b6d4, 0xec4899, 0xeab308, 0x64748b
  ];

  function getFallbackColor(name) {
    let hash = 0;
    for (let i = 0; i < name.length; i++) {
      hash = name.charCodeAt(i) + ((hash << 5) - hash);
    }
    const idx = Math.abs(hash) % DEFAULT_COLORS.length;
    return DEFAULT_COLORS[idx];
  }

  // --- Three.js Setup ---
  function initThree() {
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0f1117);

    // Coordinate convention: Z-up, X-forward, Y-left
    camera = new THREE.PerspectiveCamera(45, container.clientWidth / container.clientHeight, 0.05, 50);
    camera.up.set(0, 0, 1);
    camera.position.set(0.85, -0.75, 0.65);

    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, preserveDrawingBuffer: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    container.appendChild(renderer.domElement);

    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.target.set(0.12, 0.0, 0.12);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.maxPolarAngle = Math.PI / 2 + 0.1; // Don't dip far below ground

    // Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.65);
    scene.add(ambientLight);

    const dirLight1 = new THREE.DirectionalLight(0xffffff, 0.85);
    dirLight1.position.set(3, -4, 5);
    dirLight1.castShadow = true;
    dirLight1.shadow.mapSize.width = 1024;
    dirLight1.shadow.mapSize.height = 1024;
    scene.add(dirLight1);

    const dirLight2 = new THREE.DirectionalLight(0x60a5fa, 0.35);
    dirLight2.position.set(-3, 3, 2);
    scene.add(dirLight2);

    // Ground Grid in XY plane (Z = 0)
    const gridHelper = new THREE.GridHelper(6, 24, 0x3b82f6, 0x262a38);
    gridHelper.rotation.x = Math.PI / 2; // Lie on Z = 0
    gridHelper.position.z = 0;
    scene.add(gridHelper);

    // World Axes Helper (X = Red, Y = Green, Z = Blue)
    const axesHelper = new THREE.AxesHelper(0.4);
    axesHelper.position.set(0, 0, 0.001);
    scene.add(axesHelper);

    // Root group for all robot links
    robotRootGroup = new THREE.Group();
    robotRootGroup.name = "robot_root";
    scene.add(robotRootGroup);

    window.addEventListener('resize', onWindowResize);
    if (renderer && scene && camera) {
      renderer.render(scene, camera);
    }
    animate();
  }

  function onWindowResize() {
    if (!renderer || !camera) return;
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
  }

  function animate() {
    requestAnimationFrame(animate);

    const now = performance.now();
    const dt = Math.min((now - lastFrameTime) / 1000.0, 0.1);
    lastFrameTime = now;

    // Smooth exponential interpolation toward target transforms at 60 fps
    // Lerp rate scales with user's animationSpeed setting
    const lerpRate = Math.min(1.0, dt * (5.0 + 8.0 * animationSpeed));
    for (const linkObj of linkObjects.values()) {
      if (linkObj.hasInitialPose && linkObj.targetPos) {
        linkObj.currentPos.lerp(linkObj.targetPos, lerpRate);
        linkObj.currentQuat.slerp(linkObj.targetQuat, lerpRate);
        linkObj.currentScale.lerp(linkObj.targetScale, lerpRate);
        linkObj.matrix.compose(linkObj.currentPos, linkObj.currentQuat, linkObj.currentScale);
        linkObj.matrixWorldNeedsUpdate = true;

        if (linkObj.name === selectedLinkName) {
          displayMatrixDetails(selectedLinkName, linkObj.matrix.elements);
        }
      }
    }

    if (controls) controls.update();
    if (renderer && scene && camera) {
      renderer.render(scene, camera);
    }
  }

  // --- URDF Parsing & Visual Geometry Construction ---
  function parseNumbers(str, count) {
    if (!str) return Array(count).fill(0);
    const nums = str.trim().split(/\s+/).map(Number);
    if (nums.length < count) {
      while (nums.length < count) nums.push(0);
    }
    return nums.slice(0, count);
  }

  function parseUrdf(urdfXml) {
    const parser = new DOMParser();
    const doc = parser.parseFromString(urdfXml, "text/xml");
    const robotEl = doc.querySelector("robot");
    if (!robotEl) return new Map();

    // Map top-level named materials
    const topMaterials = new Map();
    for (const matEl of robotEl.children) {
      if (matEl.tagName.toLowerCase() !== "material") continue;
      const matName = matEl.getAttribute("name");
      const colorEl = matEl.querySelector("color");
      if (matName && colorEl && colorEl.getAttribute("rgba")) {
        topMaterials.set(matName, parseNumbers(colorEl.getAttribute("rgba"), 4));
      }
    }

    const linkDefs = new Map();

    // Iterate over direct child links
    for (const linkEl of robotEl.children) {
      if (linkEl.tagName.toLowerCase() !== "link") continue;
      const linkName = linkEl.getAttribute("name");
      if (!linkName) continue;

      let visualData = null;
      const visEl = linkEl.querySelector(":scope > visual");
      if (visEl) {
        // Origin
        const origEl = visEl.querySelector(":scope > origin");
        const xyz = origEl ? parseNumbers(origEl.getAttribute("xyz"), 3) : [0, 0, 0];
        const rpy = origEl ? parseNumbers(origEl.getAttribute("rpy"), 3) : [0, 0, 0];

        // Geometry
        let geomData = null;
        const geomEl = visEl.querySelector(":scope > geometry");
        if (geomEl && geomEl.firstElementChild) {
          const childTag = geomEl.firstElementChild.tagName.toLowerCase();
          if (childTag === "box") {
            const size = parseNumbers(geomEl.firstElementChild.getAttribute("size"), 3);
            geomData = { type: "box", size };
          } else if (childTag === "cylinder") {
            const radius = parseFloat(geomEl.firstElementChild.getAttribute("radius")) || 0.05;
            const length = parseFloat(geomEl.firstElementChild.getAttribute("length")) || 0.1;
            geomData = { type: "cylinder", radius, length };
          } else if (childTag === "sphere") {
            const radius = parseFloat(geomEl.firstElementChild.getAttribute("radius")) || 0.05;
            geomData = { type: "sphere", radius };
          } else if (childTag === "mesh") {
            const filename = geomEl.firstElementChild.getAttribute("filename") || "";
            const scale = parseNumbers(geomEl.firstElementChild.getAttribute("scale") || "1 1 1", 3);
            geomData = { type: "mesh", filename, scale };
          }
        }

        // Material color (local or top-level reference)
        let colorRgba = null;
        const matEl = visEl.querySelector(":scope > material");
        if (matEl) {
          const colorEl = matEl.querySelector("color");
          if (colorEl && colorEl.getAttribute("rgba")) {
            colorRgba = parseNumbers(colorEl.getAttribute("rgba"), 4);
          } else if (matEl.getAttribute("name") && topMaterials.has(matEl.getAttribute("name"))) {
            colorRgba = topMaterials.get(matEl.getAttribute("name"));
          }
        }

        visualData = { xyz, rpy, geom: geomData, color: colorRgba };
      }

      linkDefs.set(linkName, { name: linkName, visual: visualData });
    }

    return linkDefs;
  }

  function createVisualMesh(linkName, visualData) {
    if (!visualData || !visualData.geom) {
      // Invisible frame helper
      const group = new THREE.Group();
      const axis = new THREE.AxesHelper(0.08);
      group.add(axis);
      return group;
    }

    let geometry = null;
    const g = visualData.geom;

    if (g.type === "box") {
      geometry = new THREE.BoxGeometry(g.size[0], g.size[1], g.size[2]);
    } else if (g.type === "cylinder") {
      geometry = new THREE.CylinderGeometry(g.radius, g.radius, g.length, 32);
      // URDF cylinders are along Z; Three.js is along Y. Rotate geometry 90 deg around X.
      geometry.rotateX(Math.PI / 2);
    } else if (g.type === "sphere") {
      geometry = new THREE.SphereGeometry(g.radius, 32, 16);
    } else if (g.type === "mesh") {
      // Mesh fallback proxy: sleek rounded box + coordinate triad
      const fallbackSize = [0.12, 0.12, 0.12];
      geometry = new THREE.BoxGeometry(fallbackSize[0], fallbackSize[1], fallbackSize[2]);
    }

    // Material
    let matColor = getFallbackColor(linkName);
    let opacity = 1.0;
    if (visualData.color && visualData.color.length >= 4) {
      const [r, gr, b, a] = visualData.color;
      matColor = new THREE.Color(r, gr, b);
      opacity = a;
    }

    const material = new THREE.MeshStandardMaterial({
      color: matColor,
      roughness: 0.35,
      metalness: 0.2,
      opacity: opacity,
      transparent: opacity < 1.0
    });

    // 8x8 Checkered Chess Board Texture
    if (linkName === 'board_surface') {
      const canvas = document.createElement('canvas');
      canvas.width = 512;
      canvas.height = 512;
      const ctx = canvas.getContext('2d');
      const tileSize = 512 / 8;
      for (let r = 0; r < 8; r++) {
        for (let c = 0; c < 8; c++) {
          ctx.fillStyle = (r + c) % 2 === 0 ? '#edd6b0' : '#7b4c2b';
          ctx.fillRect(c * tileSize, r * tileSize, tileSize, tileSize);
        }
      }
      const boardTexture = new THREE.CanvasTexture(canvas);
      material.map = boardTexture;
      material.roughness = 0.45;
      material.needsUpdate = true;
    }

    const mesh = new THREE.Mesh(geometry, material);
    mesh.castShadow = true;
    mesh.receiveShadow = true;

    // Apply visual origin offset relative to link frame
    const ox = visualData.xyz[0];
    const oy = visualData.xyz[1];
    const oz = visualData.xyz[2];
    mesh.position.set(ox, oy, oz);

    // Fixed-axis roll-pitch-yaw: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    // In Three.js, Euler with 'ZYX' applies Rx, then Ry, then Rz!
    const [roll, pitch, yaw] = visualData.rpy;
    mesh.rotation.setFromEuler(new THREE.Euler(roll, pitch, yaw, 'ZYX'));

    const group = new THREE.Group();
    group.add(mesh);

    // Articulated Arm Spherical Ball Joints (balls at each joint pivot)
    if (linkName === 'arm_base_link' || linkName.includes('shoulder') ||
        linkName.includes('upper_arm') || linkName.includes('forearm') ||
        linkName.includes('wrist')) {
      const ballRadius = linkName === 'arm_base_link' ? 0.048 :
                         linkName.includes('shoulder') ? 0.040 :
                         linkName.includes('upper') ? 0.034 :
                         linkName.includes('forearm') ? 0.028 : 0.022;
      const ballGeom = new THREE.SphereGeometry(ballRadius, 24, 16);
      const ballMat = new THREE.MeshStandardMaterial({
        color: 0xd8dde8,
        metalness: 0.85,
        roughness: 0.2
      });
      const ballMesh = new THREE.Mesh(ballGeom, ballMat);
      ballMesh.castShadow = true;
      group.add(ballMesh);
    }

    // Spherical heads & collar on chess pieces
    if (linkName.startsWith('piece_')) {
      const headRadius = linkName.includes('king') ? 0.015 : 0.011;
      const headGeom = new THREE.SphereGeometry(headRadius, 20, 16);
      const headMesh = new THREE.Mesh(headGeom, material);
      const cylLen = g.length || 0.036;
      headMesh.position.set(ox, oy, oz + cylLen / 2 + headRadius * 0.7);
      headMesh.castShadow = true;
      group.add(headMesh);

      if (linkName === 'piece_pawn_e2') {
        const collarGeom = new THREE.TorusGeometry(0.011, 0.002, 12, 24);
        collarGeom.rotateX(Math.PI / 2);
        const collarMat = new THREE.MeshStandardMaterial({ color: 0xf59e0b, metalness: 0.85, roughness: 0.2 });
        const collarMesh = new THREE.Mesh(collarGeom, collarMat);
        collarMesh.position.set(ox, oy, oz + cylLen / 2);
        group.add(collarMesh);
      }
    }

    // Add axis indicator to key links only
    if (linkName === 'arm_base_link' || linkName === 'table_base' || linkName === 'gripper_base') {
      const axes = new THREE.AxesHelper(0.06);
      group.add(axes);
    }

    return group;
  }

  function rebuildRobotScene(urdfXml) {
    activeRobotDescription = urdfXml;
    const linkDefs = parseUrdf(urdfXml);

    // 1. Purge stale link objects not in new URDF
    for (const [name, obj] of linkObjects.entries()) {
      if (!linkDefs.has(name)) {
        robotRootGroup.remove(obj);
        disposeObject(obj);
        linkObjects.delete(name);
      }
    }

    // 2. Create or update link objects
    for (const [name, def] of linkDefs.entries()) {
      let linkGroup = linkObjects.get(name);
      if (linkGroup) {
        // Remove old visual children
        while (linkGroup.children.length > 0) {
          const child = linkGroup.children[0];
          linkGroup.remove(child);
          disposeObject(child);
        }
      } else {
        linkGroup = new THREE.Group();
        linkGroup.name = name;
        linkGroup.matrixAutoUpdate = false; // Essential for /xform_world column-major matrices!
        linkObjects.set(name, linkGroup);
        robotRootGroup.add(linkGroup);
      }

      const visualNode = createVisualMesh(name, def.visual);
      linkGroup.add(visualNode);
    }

    updateLinkCountBadge();
    if (renderer && scene && camera) {
      renderer.render(scene, camera);
    }
  }

  function disposeObject(obj) {
    if (!obj) return;
    obj.traverse((child) => {
      if (child.isMesh) {
        if (child.geometry) child.geometry.dispose();
        if (child.material) {
          if (Array.isArray(child.material)) child.material.forEach((m) => m.dispose());
          else child.material.dispose();
        }
      }
    });
  }

  // --- Real-time Updates from /xform_world ---
  function updateWorldTransforms(transforms) {
    if (!Array.isArray(transforms)) return;

    const seenFrames = new Set();
    for (const item of transforms) {
      const linkName = item.child_frame_id;
      seenFrames.add(linkName);
      const matrixArr = item.matrix; // 16 column-major values

      let linkObj = linkObjects.get(linkName);
      if (!linkObj) {
        // If link was not yet created from URDF, create default placeholder
        linkObj = new THREE.Group();
        linkObj.name = linkName;
        linkObj.matrixAutoUpdate = false;
        const fallbackGeom = new THREE.BoxGeometry(0.08, 0.08, 0.08);
        const fallbackMat = new THREE.MeshStandardMaterial({
          color: getFallbackColor(linkName),
          roughness: 0.4
        });
        const m = new THREE.Mesh(fallbackGeom, fallbackMat);
        linkObj.add(m);
        linkObj.add(new THREE.AxesHelper(0.08));
        robotRootGroup.add(linkObj);
        linkObjects.set(linkName, linkObj);
        updateLinkCountBadge();
      }

      if (!linkObj.targetPos) {
        linkObj.targetPos = new THREE.Vector3();
        linkObj.targetQuat = new THREE.Quaternion();
        linkObj.targetScale = new THREE.Vector3(1, 1, 1);
        linkObj.currentPos = new THREE.Vector3();
        linkObj.currentQuat = new THREE.Quaternion();
        linkObj.currentScale = new THREE.Vector3(1, 1, 1);
        linkObj.hasInitialPose = false;
      }

      // Decompose target matrix into position, quaternion, scale
      _tmpMatrix.fromArray(matrixArr);
      _tmpMatrix.decompose(linkObj.targetPos, linkObj.targetQuat, linkObj.targetScale);

      // On first pose or instant reset, snap directly
      if (!linkObj.hasInitialPose) {
        linkObj.currentPos.copy(linkObj.targetPos);
        linkObj.currentQuat.copy(linkObj.targetQuat);
        linkObj.currentScale.copy(linkObj.targetScale);
        linkObj.matrix.fromArray(matrixArr);
        linkObj.matrixWorldNeedsUpdate = true;
        linkObj.hasInitialPose = true;
      }

      // Update Inspector if this link is selected
      if (linkName === selectedLinkName) {
        displayMatrixDetails(linkName, linkObj.matrix.elements);
      }
    }

    // Default select first link if none selected
    if (!selectedLinkName && transforms.length > 0) {
      selectedLinkName = transforms[0].child_frame_id;
      displayMatrixDetails(selectedLinkName, transforms[0].matrix);
    }

    if (renderer && scene && camera) {
      renderer.render(scene, camera);
    }
  }

  function displayMatrixDetails(name, matrixArr) {
    if (selectedLinkNameEl) selectedLinkNameEl.textContent = name;
    if (!matrixGridEl || !matrixArr || matrixArr.length !== 16) return;

    // Clear grid
    matrixGridEl.innerHTML = '';
    // Display in standard 4x4 row layout for reading: row r, col c = matrixArr[c*4 + r]
    for (let r = 0; r < 4; r++) {
      for (let c = 0; c < 4; c++) {
        const val = matrixArr[c * 4 + r];
        const cell = document.createElement('div');
        cell.className = 'matrix-cell';
        if (c === 3 && r < 3) {
          cell.classList.add('translation'); // highlight tx, ty, tz
        }
        cell.textContent = val.toFixed(3);
        matrixGridEl.appendChild(cell);
      }
    }

    if (posWorldEl) {
      const tx = matrixArr[12].toFixed(3);
      const ty = matrixArr[13].toFixed(3);
      const tz = matrixArr[14].toFixed(3);
      posWorldEl.textContent = `X: ${tx}  Y: ${ty}  Z: ${tz}`;
    }
  }

  function updateLinkCountBadge() {
    if (linkCountBadge) {
      linkCountBadge.textContent = `${linkObjects.size} links`;
    }
  }

  // --- Joint States Telemetry ---
  function updateJointStates(msg) {
    if (!jointListEl || !msg.name || !msg.position) return;
    jointListEl.innerHTML = '';
    for (let i = 0; i < msg.name.length; i++) {
      const name = msg.name[i];
      const pos = msg.position[i];
      const item = document.createElement('div');
      item.className = 'joint-item';
      item.innerHTML = `<span>${name}</span><span class="joint-val">${pos.toFixed(3)}</span>`;
      jointListEl.appendChild(item);
    }
  }

  // --- FSM Status Update ---
  function updateFsmStatus(msg) {
    activeFsmStatus = msg;
    if (fsmStateBadge) fsmStateBadge.textContent = msg.current_state || 'none';
    if (fsmRunningTag) {
      if (msg.running) {
        fsmRunningTag.textContent = 'RUNNING';
        fsmRunningTag.className = 'fsm-running-tag running';
        btnStartFsm.disabled = true;
        btnPauseFsm.disabled = false;
      } else {
        fsmRunningTag.textContent = 'STOPPED';
        fsmRunningTag.className = 'fsm-running-tag stopped';
        btnStartFsm.disabled = false;
        btnPauseFsm.disabled = true;
      }
    }
    if (fsmTimeInState) fsmTimeInState.textContent = `${(msg.time_in_state || 0).toFixed(2)} s`;
    if (fsmLastTransition) fsmLastTransition.textContent = msg.last_transition || 'none';
  }

  // --- Server-Sent Events (SSE) Client ---
  function initSse() {
    if (sseSource) {
      sseSource.close();
    }

    setConnectionStatus(false, 'Connecting...');
    sseSource = new EventSource('/stream');

    sseSource.onopen = () => {
      setConnectionStatus(true, 'Connected (SSE)');
      // Fetch robot description and status
      fetchStatusAndRobot();
    };

    sseSource.onmessage = (event) => {
      try {
        const msg = JSON.parse(event.data);
        if (msg.op === "publish") {
          if (msg.topic === "/xform_world") {
            updateWorldTransforms(msg.msg.transforms);
          } else if (msg.topic === "/fsm/status") {
            updateFsmStatus(msg.msg);
          } else if (msg.topic === "/joint_states") {
            updateJointStates(msg.msg);
          }
        } else if (msg.op === "connected") {
          if (msg.backend_connected) {
            setConnectionStatus(true, 'Connected to 127.0.0.1:9095');
          } else {
            setConnectionStatus(false, 'Waiting for make demo...');
          }
        }
      } catch (err) {
        console.error("Error processing SSE message:", err);
      }
    };

    sseSource.onerror = () => {
      setConnectionStatus(false, 'Disconnected');
    };
  }

  function setConnectionStatus(connected, text) {
    if (connDot) {
      connDot.className = connected ? 'status-dot connected' : 'status-dot';
    }
    if (connText) {
      connText.textContent = text;
    }
  }

  // --- API Calls ---
  async function fetchStatusAndRobot() {
    try {
      const res = await fetch('/api/status');
      const data = await res.json();
      activeRobotStatus = data.description_status;

      // Populate robot dropdown
      const robRes = await fetch('/api/robot_list');
      const robData = await robRes.json();
      if (robotSelect && robData.robots) {
        robotSelect.innerHTML = '';
        let defaultIdx = 0;
        for (let i = 0; i < robData.robots.length; i++) {
          const r = robData.robots[i];
          const opt = document.createElement('option');
          opt.value = r.path;
          opt.textContent = r.name;
          if (r.path.includes('chess_arm')) defaultIdx = i;
          robotSelect.appendChild(opt);
        }
        if (robotSelect.options.length > defaultIdx) {
          robotSelect.selectedIndex = defaultIdx;
        }
      }

      // Populate FSM dropdown
      const fsmRes = await fetch('/api/fsm_list');
      const fsmData = await fsmRes.json();
      if (fsmSelect && fsmData.fsm_files) {
        fsmSelect.innerHTML = '';
        let defaultFsmIdx = 0;
        for (let i = 0; i < fsmData.fsm_files.length; i++) {
          const f = fsmData.fsm_files[i];
          const opt = document.createElement('option');
          opt.value = f.identifier;
          opt.textContent = f.identifier;
          if (f.identifier.includes('chess')) defaultFsmIdx = i;
          fsmSelect.appendChild(opt);
        }
        if (fsmSelect.options.length > defaultFsmIdx) {
          fsmSelect.selectedIndex = defaultFsmIdx;
        }
      }

      // Fetch active description
      const descRes = await fetch('/api/robot_description');
      const descData = await descRes.json();
      if (descData.description) {
        rebuildRobotScene(descData.description);
      }
    } catch (e) {
      console.warn("Could not fetch status/robot:", e);
    }
  }

  async function loadSelectedRobot() {
    const relPath = robotSelect.value;
    if (!relPath) return;
    try {
      const res = await fetch('/api/load_robot', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ path: relPath })
      });
      const data = await res.json();
      if (data.result) {
        // Fetch new description
        const descRes = await fetch('/api/robot_description');
        const descData = await descRes.json();
        if (descData.description) {
          rebuildRobotScene(descData.description);
        }
      } else {
        alert("Failed to load robot: " + (data.status?.error || "unknown error"));
      }
    } catch (e) {
      alert("Error loading robot: " + e.message);
    }
  }

  async function loadSelectedFsm() {
    const ident = fsmSelect.value;
    if (!ident) return;
    try {
      const res = await fetch('/api/load_fsm', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ identifier: ident })
      });
      const data = await res.json();
      if (!data.result) {
        alert("Failed to load FSM: " + JSON.stringify(data.values));
      }
    } catch (e) {
      alert("Error loading FSM: " + e.message);
    }
  }

  async function setFsmRunning(running) {
    try {
      await fetch('/api/set_fsm_running', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ running })
      });
    } catch (e) {
      console.error("Error setting FSM running state:", e);
    }
  }

  async function resetJoints() {
    try {
      await fetch('/api/reset_joints', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({})
      });
    } catch (e) {
      console.error("Error resetting joints:", e);
    }
  }

  // --- Video Recording Feature ---
  function toggleRecording() {
    if (isRecording) {
      stopRecording();
    } else {
      startRecording();
    }
  }

  function startRecording() {
    if (!renderer || !renderer.domElement) return;
    const canvas = renderer.domElement;
    const stream = canvas.captureStream(30); // 30 fps capture

    let options = { mimeType: 'video/webm; codecs=vp9' };
    if (!MediaRecorder.isTypeSupported(options.mimeType)) {
      options = { mimeType: 'video/webm' };
      if (!MediaRecorder.isTypeSupported(options.mimeType)) {
        options = { mimeType: '' };
      }
    }

    try {
      mediaRecorder = new MediaRecorder(stream, options);
    } catch (e) {
      alert("MediaRecorder not supported: " + e.message);
      return;
    }

    recordedChunks = [];
    mediaRecorder.ondataavailable = (e) => {
      if (e.data && e.data.size > 0) {
        recordedChunks.push(e.data);
      }
    };

    mediaRecorder.onstop = () => {
      const blob = new Blob(recordedChunks, { type: 'video/webm' });
      // Post to server to update portfolio video automatically
      fetch('/api/save_recording', {
        method: 'POST',
        headers: { 'Content-Type': 'video/webm' },
        body: blob
      }).catch(e => console.log("save_recording notice:", e));

      // Trigger download if interactive
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `choreography_${Date.now()}.webm`;
      document.body.appendChild(a);
      a.click();
      setTimeout(() => {
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
      }, 100);
    };

    mediaRecorder.start();
    isRecording = true;
    recordingStartTime = Date.now();
    btnRecord.classList.add('recording');
    recordLabel.textContent = "Stop & Save Video";

    recordTimerInterval = setInterval(() => {
      const sec = ((Date.now() - recordingStartTime) / 1000).toFixed(1);
      recordLabel.textContent = `REC (${sec}s) - Stop`;
    }, 200);
  }

  function stopRecording() {
    if (!mediaRecorder || !isRecording) return;
    mediaRecorder.stop();
    isRecording = false;
    clearInterval(recordTimerInterval);
    btnRecord.classList.remove('recording');
    recordLabel.textContent = "Record Video";
  }

  // --- Attach Event Handlers ---
  function attachEvents() {
    if (robotSelect) robotSelect.addEventListener('change', loadSelectedRobot);
    if (fsmSelect) fsmSelect.addEventListener('change', loadSelectedFsm);

    if (btnStartFsm) btnStartFsm.addEventListener('click', () => setFsmRunning(true));
    if (btnPauseFsm) btnPauseFsm.addEventListener('click', () => setFsmRunning(false));
    if (btnResetJoints) btnResetJoints.addEventListener('click', resetJoints);
    if (btnRecord) btnRecord.addEventListener('click', toggleRecording);

    // Animation Speed Controls
    function setAnimationSpeed(speed, updateBackend = true) {
      animationSpeed = Math.max(0.1, Math.min(3.0, speed));
      if (speedSlider) speedSlider.value = animationSpeed.toFixed(1);
      if (speedValueLabel) {
        let labelDesc = "Normal";
        if (animationSpeed <= 0.35) labelDesc = "Very Slow";
        else if (animationSpeed <= 0.75) labelDesc = "Slow & Smooth";
        else if (animationSpeed >= 1.8) labelDesc = "Fast";
        else if (animationSpeed >= 1.25) labelDesc = "Brisk";
        speedValueLabel.textContent = `${animationSpeed.toFixed(1)}x (${labelDesc})`;
      }

      // Update active preset button highlight
      document.querySelectorAll('.preset-btn').forEach((btn) => {
        const btnSpeed = parseFloat(btn.dataset.speed);
        if (Math.abs(btnSpeed - animationSpeed) < 0.08) {
          btn.classList.add('active');
        } else {
          btn.classList.remove('active');
        }
      });

      if (updateBackend) {
        fetch('/api/set_fsm_speed', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ speed: animationSpeed })
        }).catch((err) => console.log("[Viewer] set_fsm_speed notice:", err));
      }
    }

    if (speedSlider) {
      speedSlider.addEventListener('input', (e) => {
        setAnimationSpeed(parseFloat(e.target.value), false);
      });
      speedSlider.addEventListener('change', (e) => {
        setAnimationSpeed(parseFloat(e.target.value), true);
      });
    }

    document.querySelectorAll('.preset-btn').forEach((btn) => {
      btn.addEventListener('click', () => {
        const sp = parseFloat(btn.dataset.speed);
        if (!isNaN(sp)) {
          setAnimationSpeed(sp, true);
        }
      });
    });

    // Click on canvas to select links
    const raycaster = new THREE.Raycaster();
    const mouse = new THREE.Vector2();
    container.addEventListener('pointerdown', (e) => {
      const rect = container.getBoundingClientRect();
      mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
      mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
      raycaster.setFromCamera(mouse, camera);
      const intersects = raycaster.intersectObjects(robotRootGroup.children, true);
      if (intersects.length > 0) {
        // Find ancestor group representing link
        let curr = intersects[0].object;
        while (curr && curr.parent !== robotRootGroup && curr.parent !== scene) {
          curr = curr.parent;
        }
        if (curr && curr.name) {
          selectedLinkName = curr.name;
          displayMatrixDetails(curr.name, curr.matrix.elements);
        }
      }
    });
  }

  function checkUrlParams() {
    const params = new URLSearchParams(window.location.search);
    if (params.get('auto_dance')) {
      setTimeout(async () => {
        console.log("[Viewer] Executing auto_dance...");
        // Load Fetch robot
        await fetch('/api/load_robot', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ path: 'fetch/fetch.urdf' })
        });
        // Load fetch_dance
        await fetch('/api/load_fsm', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ identifier: 'fetch_dance' })
        });
        // Start running
        await fetch('/api/set_fsm_running', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ running: true })
        });

        const recSec = parseInt(params.get('auto_record') || '0', 10);
        if (recSec > 0) {
          console.log(`[Viewer] Starting auto_record for ${recSec} seconds...`);
          startRecording();
          setTimeout(() => {
            console.log("[Viewer] Stopping auto_record...");
            stopRecording();
          }, recSec * 1000);
        }

        if (params.get('capture_frames')) {
          console.log("[Viewer] capture_frames requested...");
          setTimeout(() => {
            captureChoreographySequence(60);
          }, 1500);
        }
      }, 1000);
    }
  }

  async function captureChoreographySequence(numFrames = 60) {
    console.log(`[Capture] Recording ${numFrames} frames from WebGL canvas...`);
    for (let f = 0; f < numFrames; f++) {
      if (renderer && scene && camera) {
        renderer.render(scene, camera);
        const dataUrl = renderer.domElement.toDataURL('image/png');
        await fetch('/api/save_frame', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ index: f, data: dataUrl })
        });
      }
      await new Promise(r => setTimeout(r, 90));
    }
    console.log("[Capture] Compiling video with ffmpeg via /api/compile_frames...");
    await fetch('/api/compile_frames', { method: 'POST' });
    console.log("[Capture] Compilation complete!");
  }
  window.captureChoreographySequence = captureChoreographySequence;

  // --- Bootstrap ---
  window.addEventListener('DOMContentLoaded', () => {
    initThree();
    attachEvents();
    fetchStatusAndRobot();
    initSse();
    checkUrlParams();
  });
})();
