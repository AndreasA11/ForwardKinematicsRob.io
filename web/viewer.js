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

  // External Robot Gateway DOM Elements
  const extProtocolSelect = document.getElementById('extProtocolSelect');
  const extHostInput = document.getElementById('extHostInput');
  const extPortInput = document.getElementById('extPortInput');
  const btnToggleExtConnect = document.getElementById('btnToggleExtConnect');
  const extConnDot = document.getElementById('extConnDot');
  const extConnStatusText = document.getElementById('extConnStatusText');
  const extActiveProtocolBadge = document.getElementById('extActiveProtocolBadge');
  const mbotControlsDeck = document.getElementById('mbotControlsDeck');
  const mbotPoseDisplay = document.getElementById('mbotPoseDisplay');
  const mbotGoalX = document.getElementById('mbotGoalX');
  const mbotGoalY = document.getElementById('mbotGoalY');
  const mbotGoalTheta = document.getElementById('mbotGoalTheta');
  const btnMbotSendGoal = document.getElementById('btnMbotSendGoal');
  const btnPresetSquare = document.getElementById('btnPresetSquare');
  const btnPresetFwdBack = document.getElementById('btnPresetFwdBack');
  const btnPresetRotate = document.getElementById('btnPresetRotate');
  const btnMbotEmergencyStop = document.getElementById('btnMbotEmergencyStop');
  const btnMbotFetchMap = document.getElementById('btnMbotFetchMap');
  const rx200ControlsDeck = document.getElementById('rx200ControlsDeck');
  const rx200ArmStatus = document.getElementById('rx200ArmStatus');
  const btnArmHome = document.getElementById('btnArmHome');
  const btnArmSleep = document.getElementById('btnArmSleep');
  const btnArmRelease = document.getElementById('btnArmRelease');
  const btnArmGrasp = document.getElementById('btnArmGrasp');
  const btnArmChessPickAndPlace = document.getElementById('btnArmChessPickAndPlace');
  let externalConnected = false;

  // --- Color Palette for Fallback ---
  const DEFAULT_COLORS = [
    0x3b82f6, 0x10b981, 0xf59e0b, 0x8b5cf6,
    0x06b6d4, 0xec4899, 0xeab308, 0x64748b
  ];

  function getFallbackColor(name) {
    const n = (name || '').toLowerCase();
    if (n.includes('finger') || n.includes('jaw') || n === 'gripper_left_joint' || n === 'gripper_right_joint') {
      return 0xf59e0b; // Radiant amber gold for gripper fingers!
    }
    if (n.includes('upper_arm') || n.includes('forearm')) {
      return 0x2563eb; // Royal electric cobalt blue!
    }
    if (n.includes('shoulder')) {
      return 0x3b82f6; // High-tech accent blue!
    }
    if (n.includes('wrist')) {
      return 0x1e293b; // Deep metallic slate for wrist!
    }
    if (n.includes('gripper') || n.includes('palm')) {
      return 0x334155; // Dark titanium charcoal!
    }
    if (n.includes('base_link') || n.includes('pedestal')) {
      return 0x1f242d; // Matte pedestal slate!
    }
    if (n.includes('table')) {
      return 0x181c24; // Base table slate!
    }
    if (n.includes('board') || n.includes('chess')) {
      return 0x5c3a21; // Rich polished mahogany/walnut!
    }
    if (n.includes('white') || n.includes('pawn_e2') || n.includes('pawn_d2') || n.includes('king_white') || n.includes('knight_white')) {
      return 0xf4efe2; // Ivory porcelain white piece!
    }
    if (n.includes('black') || n.includes('pawn_e7') || n.includes('king_black')) {
      return 0x1e2026; // Ebony obsidian black piece!
    }
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

    // World Axes Helper (X = Red, Y = Green, Z = Blue) - Subtle 15cm triad
    const axesHelper = new THREE.AxesHelper(0.15);
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

  function getChildTag(el, tag) {
    if (!el || !el.children) return null;
    const t = tag.toLowerCase();
    for (let i = 0; i < el.children.length; i++) {
      if (el.children[i].tagName && el.children[i].tagName.toLowerCase() === t) return el.children[i];
    }
    return null;
  }

  function parseUrdf(urdfXml) {
    const parser = new DOMParser();
    const doc = parser.parseFromString(urdfXml, "text/xml");
    const robotEl = getChildTag(doc, "robot") || doc.querySelector("robot");
    if (!robotEl) return new Map();

    // Map top-level named materials
    const topMaterials = new Map();
    for (let i = 0; i < robotEl.children.length; i++) {
      const matEl = robotEl.children[i];
      if (matEl.tagName.toLowerCase() !== "material") continue;
      const matName = matEl.getAttribute("name");
      const colorEl = getChildTag(matEl, "color") || matEl.querySelector("color");
      if (matName && colorEl && colorEl.getAttribute("rgba")) {
        topMaterials.set(matName, parseNumbers(colorEl.getAttribute("rgba"), 4));
      }
    }

    const linkDefs = new Map();

    for (let i = 0; i < robotEl.children.length; i++) {
      const linkEl = robotEl.children[i];
      if (linkEl.tagName.toLowerCase() !== "link") continue;
      const linkName = linkEl.getAttribute("name");
      if (!linkName) continue;

      let visualData = null;
      let visEl = getChildTag(linkEl, "visual");
      if (!visEl) {
        try { visEl = linkEl.querySelector(":scope > visual") || linkEl.querySelector("visual"); } catch (e) {}
      }

      if (visEl) {
        let origEl = getChildTag(visEl, "origin");
        if (!origEl) {
          try { origEl = visEl.querySelector(":scope > origin") || visEl.querySelector("origin"); } catch (e) {}
        }
        const xyz = origEl ? parseNumbers(origEl.getAttribute("xyz"), 3) : [0, 0, 0];
        const rpy = origEl ? parseNumbers(origEl.getAttribute("rpy"), 3) : [0, 0, 0];

        let geomData = null;
        let geomEl = getChildTag(visEl, "geometry");
        if (!geomEl) {
          try { geomEl = visEl.querySelector(":scope > geometry") || visEl.querySelector("geometry"); } catch (e) {}
        }
        if (geomEl && geomEl.children.length > 0) {
          const childEl = geomEl.children[0];
          const childTag = childEl.tagName.toLowerCase();
          if (childTag === "box") {
            const size = parseNumbers(childEl.getAttribute("size"), 3);
            geomData = { type: "box", size };
          } else if (childTag === "cylinder") {
            const radius = parseFloat(childEl.getAttribute("radius")) || 0.05;
            const length = parseFloat(childEl.getAttribute("length")) || 0.1;
            geomData = { type: "cylinder", radius, length };
          } else if (childTag === "sphere") {
            const radius = parseFloat(childEl.getAttribute("radius")) || 0.05;
            geomData = { type: "sphere", radius };
          } else if (childTag === "mesh") {
            const filename = childEl.getAttribute("filename") || "";
            const scale = parseNumbers(childEl.getAttribute("scale") || "1 1 1", 3);
            geomData = { type: "mesh", filename, scale };
          }
        }

        let colorRgba = null;
        let matEl = getChildTag(visEl, "material");
        if (!matEl) {
          try { matEl = visEl.querySelector(":scope > material") || visEl.querySelector("material"); } catch (e) {}
        }
        if (matEl) {
          let colorEl = getChildTag(matEl, "color");
          if (!colorEl) {
            try { colorEl = matEl.querySelector("color"); } catch (e) {}
          }
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
      return new THREE.Group();
    }

    let geometry = null;
    const g = visualData.geom;
    const n = (linkName || "").toLowerCase();

    if (g.type === "box") {
      geometry = new THREE.BoxGeometry(g.size[0], g.size[1], g.size[2]);
    } else if (g.type === "cylinder") {
      geometry = new THREE.CylinderGeometry(g.radius, g.radius, g.length, 32);
      // URDF cylinders are along Z; Three.js is along Y. Rotate geometry 90 deg around X.
      geometry.rotateX(Math.PI / 2);
    } else if (g.type === "sphere") {
      geometry = new THREE.SphereGeometry(g.radius, 32, 16);
    } else if (g.type === "mesh") {
      const fn = (g.filename || "").toLowerCase();

      // Sleek cylindrical link bodies for robot arm segments
      let geomType = "cylinder";
      let cylRadius = 0.018;
      let cylLength = 0.05;
      let boxSize = null;
      let offset = [0, 0, 0];
      let axis = "z"; // default cylinder axis along Z in robot coordinates

      if (n.includes('base_link') || fn.includes('base')) {
        cylRadius = 0.065;
        cylLength = 0.035;
        axis = "z";
        offset = [0, 0, 0.0175];
      } else if (n.includes('shoulder') || fn.includes('shoulder')) {
        cylRadius = 0.038;
        cylLength = 0.045;
        axis = "z";
        offset = [0, 0, 0.0225];
      } else if (n.includes('upper_arm') || fn.includes('upper')) {
        // Upper arm boom cylinder connecting shoulder to elbow
        cylRadius = 0.022;
        cylLength = 0.19;
        axis = "upper"; // tilted slightly toward elbow
        offset = [0.025, 0, 0.095];
      } else if (n.includes('forearm') || fn.includes('forearm')) {
        // Forearm boom cylinder connecting elbow to wrist along +X
        cylRadius = 0.018;
        cylLength = 0.18;
        axis = "x";
        offset = [0.09, 0, 0];
      } else if (n.includes('wrist') || fn.includes('wrist')) {
        // Wrist cylinder along +X
        cylRadius = 0.015;
        cylLength = 0.05;
        axis = "x";
        offset = [0.025, 0, 0];
      } else if (n.includes('gripper') && !n.includes('finger') && !n.includes('bar') && !n.includes('prop')) {
        // Gripper palm cylinder
        cylRadius = 0.020;
        cylLength = 0.035;
        axis = "x";
        offset = [0.016, 0, 0];
      } else if (n.includes('finger') || fn.includes('finger')) {
        // Gripper finger jaws
        geomType = "box";
        boxSize = [0.028, 0.007, 0.014];
        const yOff = n.includes('left') ? 0.004 : (n.includes('right') ? -0.004 : 0);
        offset = [0.014, yOff, 0];
      } else if (n.includes('bar') || n.includes('prop') || n.includes('ee_')) {
        cylRadius = 0.008;
        cylLength = 0.016;
        axis = "z";
      } else {
        cylRadius = 0.018;
        cylLength = 0.05;
        axis = "z";
      }

      if (geomType === "box" && boxSize) {
        geometry = new THREE.BoxGeometry(boxSize[0], boxSize[1], boxSize[2]);
      } else {
        geometry = new THREE.CylinderGeometry(cylRadius, cylRadius, cylLength, 24);
        if (axis === "z") {
          geometry.rotateX(Math.PI / 2);
        } else if (axis === "x") {
          geometry.rotateZ(Math.PI / 2);
        } else if (axis === "upper") {
          geometry.rotateX(Math.PI / 2);
          geometry.rotateY(0.245);
        }
      }

      const ox0 = visualData.xyz[0];
      const oy0 = visualData.xyz[1];
      const oz0 = visualData.xyz[2];
      if (ox0 === 0 && oy0 === 0 && oz0 === 0) {
        if (offset[0] !== 0 || offset[1] !== 0 || offset[2] !== 0) {
          geometry.translate(offset[0], offset[1], offset[2]);
        }
      }
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
      metalness: 0.25,
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
    const [roll, pitch, yaw] = visualData.rpy;
    mesh.rotation.setFromEuler(new THREE.Euler(roll, pitch, yaw, 'ZYX'));

    const group = new THREE.Group();
    group.add(mesh);

    // Spherical ball joints at joint pivots
    if (linkName === 'arm_base_link' || n.includes('base_link') || n.includes('shoulder') ||
        n.includes('upper_arm') || n.includes('forearm') || n.includes('wrist') ||
        (n.includes('gripper') && !n.includes('finger') && !n.includes('bar') && !n.includes('prop'))) {
      const ballRadius = (linkName === 'arm_base_link' || n.includes('base_link')) ? 0.045 :
                         n.includes('shoulder') ? 0.036 :
                         n.includes('upper') ? 0.028 :
                         n.includes('forearm') ? 0.024 :
                         n.includes('wrist') ? 0.018 : 0.015;
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
      const cylLen = (g && g.length) ? g.length : 0.036;
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

    // Subtle axis indicator on root base only
    if (linkName === 'arm_base_link' || linkName === 'table_base' || linkName === 'rx200/base_link') {
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
        const fallbackGeom = new THREE.BoxGeometry(0.035, 0.035, 0.035);
        const fallbackMat = new THREE.MeshStandardMaterial({
          color: getFallbackColor(linkName),
          roughness: 0.4
        });
        const m = new THREE.Mesh(fallbackGeom, fallbackMat);
        linkObj.add(m);
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
          } else if (msg.topic === "/robot_description") {
            if (msg.msg && msg.msg.description) {
              rebuildRobotScene(msg.msg.description);
            }
          } else if (msg.topic === "/fsm/status") {
            updateFsmStatus(msg.msg);
          } else if (msg.topic === "/joint_states") {
            updateJointStates(msg.msg);
          } else if (msg.topic === "/mbot/pose") {
            updateMbotPoseDisplay(msg.data);
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
          if (r.path.includes('rx200_chess')) defaultIdx = i;
          else if (r.path.includes('rx200') && !robData.robots[defaultIdx]?.path.includes('rx200_chess')) defaultIdx = i;
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
          if (f.identifier.includes('rx200_chess')) defaultFsmIdx = i;
          else if (f.identifier.includes('chess') && !fsmData.fsm_files[defaultFsmIdx]?.identifier.includes('rx200_chess')) defaultFsmIdx = i;
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

    // --- External Robot Gateway Event Listeners ---
    if (extProtocolSelect) {
      extProtocolSelect.addEventListener('change', () => {
        const mode = extProtocolSelect.value;
        if (mode === 'mbot') {
          if (extPortInput) extPortInput.value = '5005';
          if (mbotControlsDeck) mbotControlsDeck.style.display = 'flex';
          if (rx200ControlsDeck) rx200ControlsDeck.style.display = 'none';
          if (extActiveProtocolBadge) extActiveProtocolBadge.textContent = 'MBot Bridge';
        } else if (mode === 'rosbridge') {
          if (extPortInput) extPortInput.value = '9090';
          if (mbotControlsDeck) mbotControlsDeck.style.display = 'none';
          if (rx200ControlsDeck) rx200ControlsDeck.style.display = 'flex';
          if (extActiveProtocolBadge) extActiveProtocolBadge.textContent = 'ROSBridge';
        } else {
          if (mbotControlsDeck) mbotControlsDeck.style.display = 'none';
          if (rx200ControlsDeck) rx200ControlsDeck.style.display = 'none';
          if (extActiveProtocolBadge) extActiveProtocolBadge.textContent = 'Offline';
        }
      });
    }

    if (btnToggleExtConnect) {
      btnToggleExtConnect.addEventListener('click', async () => {
        if (externalConnected) {
          // Disconnect
          try {
            const res = await fetch('/api/external_robot/disconnect', { method: 'POST' });
            const data = await res.json();
            updateExternalRobotUI(data.status);
          } catch (err) {
            console.error("Disconnect error:", err);
          }
        } else {
          // Connect
          const mode = extProtocolSelect ? extProtocolSelect.value : 'mbot';
          const host = extHostInput ? extHostInput.value.trim() || '127.0.0.1' : '127.0.0.1';
          const port = extPortInput ? parseInt(extPortInput.value, 10) || (mode === 'mbot' ? 5005 : 9090) : (mode === 'mbot' ? 5005 : 9090);

          if (extConnDot) extConnDot.className = 'ext-status-dot connecting';
          if (extConnStatusText) extConnStatusText.textContent = `Connecting to ${mode} at ${host}:${port}...`;

          try {
            const res = await fetch('/api/external_robot/connect', {
              method: 'POST',
              headers: { 'Content-Type': 'application/json' },
              body: JSON.stringify({ mode, host, port }),
            });
            const data = await res.json();
            updateExternalRobotUI(data.status);

            if (data.result) {
              // Automatically load corresponding robot model if available
              if (mode === 'mbot' && robotSelect) {
                for (let i = 0; i < robotSelect.options.length; i++) {
                  if (robotSelect.options[i].value.includes('mbot')) {
                    robotSelect.selectedIndex = i;
                    loadSelectedRobot();
                    break;
                  }
                }
              } else if (mode === 'rosbridge' && robotSelect) {
                for (let i = 0; i < robotSelect.options.length; i++) {
                  if (robotSelect.options[i].value.includes('rx200')) {
                    robotSelect.selectedIndex = i;
                    loadSelectedRobot();
                    break;
                  }
                }
              }
            } else {
              alert(data.message || "Failed to connect to external robot");
            }
          } catch (err) {
            console.error("Connect error:", err);
            if (extConnDot) extConnDot.className = 'ext-status-dot error';
            if (extConnStatusText) extConnStatusText.textContent = `Connection error: ${err.message}`;
          }
        }
      });
    }

    // MBot Navigation Actions
    if (btnMbotSendGoal) {
      btnMbotSendGoal.addEventListener('click', async () => {
        const gx = parseFloat(mbotGoalX ? mbotGoalX.value : 0) || 0;
        const gy = parseFloat(mbotGoalY ? mbotGoalY.value : 0) || 0;
        const gth = parseFloat(mbotGoalTheta ? mbotGoalTheta.value : 0) || 0;
        try {
          const res = await fetch('/api/external_robot/mbot/navigate', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ waypoints: [{ x: gx, y: gy, theta: gth }] }),
          });
          const data = await res.json();
          if (data.result) {
            console.log(`[MBot] Sent goal waypoint: (${gx}, ${gy}, ${gth})`);
          } else {
            alert(data.message || "Failed to send navigation path");
          }
        } catch (err) {
          alert(`Navigation request error: ${err.message}`);
        }
      });
    }

    if (btnPresetSquare) {
      btnPresetSquare.addEventListener('click', async () => {
        const squarePath = [
          { x: 1.0, y: 0.0, theta: 0.0 },
          { x: 1.0, y: 1.0, theta: 1.5708 },
          { x: 0.0, y: 1.0, theta: 3.1416 },
          { x: 0.0, y: 0.0, theta: 0.0 },
        ];
        await fetch('/api/external_robot/mbot/navigate', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ waypoints: squarePath }),
        });
      });
    }

    if (btnPresetFwdBack) {
      btnPresetFwdBack.addEventListener('click', async () => {
        const path = [
          { x: 1.0, y: 0.0, theta: 0.0 },
          { x: 0.0, y: 0.0, theta: 3.1416 },
        ];
        await fetch('/api/external_robot/mbot/navigate', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ waypoints: path }),
        });
      });
    }

    if (btnPresetRotate) {
      btnPresetRotate.addEventListener('click', async () => {
        const path = [
          { x: 0.0, y: 0.0, theta: 1.5708 },
          { x: 0.0, y: 0.0, theta: 3.1416 },
          { x: 0.0, y: 0.0, theta: -1.5708 },
          { x: 0.0, y: 0.0, theta: 0.0 },
        ];
        await fetch('/api/external_robot/mbot/navigate', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ waypoints: path }),
        });
      });
    }

    if (btnMbotEmergencyStop) {
      btnMbotEmergencyStop.addEventListener('click', async () => {
        try {
          const res = await fetch('/api/external_robot/mbot/stop', { method: 'POST' });
          const data = await res.json();
          console.log("[MBot] Emergency stop issued:", data);
        } catch (err) {
          alert(`Stop error: ${err.message}`);
        }
      });
    }

    if (btnMbotFetchMap) {
      btnMbotFetchMap.addEventListener('click', async () => {
        try {
          const res = await fetch('/api/external_robot/mbot/get_map', { method: 'POST' });
          const data = await res.json();
          if (data.result && data.map) {
            const m = data.map;
            alert(`Map received: ${m.width}x${m.height} cells @ ${m.meters_per_cell}m/cell, Origin: (${m.origin_x}, ${m.origin_y})`);
          } else {
            alert(data.error || "No map data available from SLAM_MAP");
          }
        } catch (err) {
          alert(`Map error: ${err.message}`);
        }
      });
    }

    // Trossen RX200 Arm Actions
    function sendArmCommand(action) {
      fetch('/api/external_robot/arm/command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ action }),
      }).then(r => r.json()).then(data => {
        console.log(`[RX200 Arm] Command ${action}:`, data);
      }).catch(err => alert(`Arm command error: ${err.message}`));
    }

    if (btnArmHome) btnArmHome.addEventListener('click', () => sendArmCommand('home'));
    if (btnArmSleep) btnArmSleep.addEventListener('click', () => sendArmCommand('sleep'));
    if (btnArmRelease) btnArmRelease.addEventListener('click', () => sendArmCommand('release'));
    if (btnArmGrasp) btnArmGrasp.addEventListener('click', () => sendArmCommand('grasp'));
    if (btnArmChessPickAndPlace) {
      btnArmChessPickAndPlace.addEventListener('click', () => {
        if (robotSelect) {
          for (let i = 0; i < robotSelect.options.length; i++) {
            if (robotSelect.options[i].value.includes('rx200_chess') || robotSelect.options[i].value.includes('rx200')) {
              if (robotSelect.selectedIndex !== i) {
                robotSelect.selectedIndex = i;
                loadSelectedRobot();
              }
              break;
            }
          }
        }
        sendArmCommand('chess_pick_and_place');
      });
    }

    // Status polling
    setInterval(pollExternalRobotStatus, 1500);
  }

  function updateMbotPoseDisplay(pose) {
    if (!pose || !mbotPoseDisplay) return;
    const x = typeof pose.x === 'number' ? pose.x : 0;
    const y = typeof pose.y === 'number' ? pose.y : 0;
    const th = typeof pose.theta === 'number' ? pose.theta : 0;
    const deg = (th * 180 / Math.PI).toFixed(1);
    mbotPoseDisplay.textContent = `X: ${x.toFixed(3)} m | Y: ${y.toFixed(3)} m | θ: ${deg}°`;
  }

  function updateExternalRobotUI(status) {
    if (!status) return;
    externalConnected = Boolean(status.connected);

    if (extConnDot) {
      if (status.status === 'connected') {
        extConnDot.className = 'ext-status-dot connected';
      } else if (status.status === 'connecting') {
        extConnDot.className = 'ext-status-dot connecting';
      } else if (status.status === 'error') {
        extConnDot.className = 'ext-status-dot error';
      } else {
        extConnDot.className = 'ext-status-dot';
      }
    }

    if (extConnStatusText) {
      if (status.status === 'connected') {
        extConnStatusText.textContent = `Connected (${status.mode.toUpperCase()} @ ${status.host}:${status.port})`;
        extConnStatusText.style.color = 'var(--accent-green)';
      } else if (status.status === 'connecting') {
        extConnStatusText.textContent = `Connecting to ${status.host}:${status.port}...`;
        extConnStatusText.style.color = 'var(--accent-orange)';
      } else if (status.status === 'error') {
        extConnStatusText.textContent = status.last_error || 'Connection Failed';
        extConnStatusText.style.color = 'var(--accent-red)';
      } else {
        extConnStatusText.textContent = 'Not Connected';
        extConnStatusText.style.color = 'var(--text-muted)';
      }
    }

    if (btnToggleExtConnect) {
      if (externalConnected) {
        btnToggleExtConnect.textContent = '🔌 Disconnect from Robot';
        btnToggleExtConnect.className = 'btn btn-danger';
      } else {
        btnToggleExtConnect.textContent = '⚡ Connect to External Robot';
        btnToggleExtConnect.className = 'btn btn-primary';
      }
    }

    if (status.mbot_pose) {
      updateMbotPoseDisplay(status.mbot_pose);
    }

    if (status.mode === 'rosbridge' && status.connected) {
      if (rx200ArmStatus) {
        const j = status.arm_joints || {};
        const keys = Object.keys(j);
        if (keys.length > 0) {
          const summary = keys.map(k => `${k}: ${Number(j[k]).toFixed(2)}`).join(' | ');
          rx200ArmStatus.textContent = `Streaming (${keys.length} joints): ${summary}`;
          rx200ArmStatus.style.color = 'var(--accent-green)';
        } else {
          rx200ArmStatus.textContent = 'Connected. Waiting for /joint_states telemetry...';
          rx200ArmStatus.style.color = 'var(--accent-orange)';
        }
      }
    } else if (status.mode === 'rosbridge' && !status.connected) {
      if (rx200ArmStatus && status.status === 'error') {
        rx200ArmStatus.textContent = status.last_error || 'Connection failed';
        rx200ArmStatus.style.color = 'var(--accent-red)';
      }
    }
  }

  async function pollExternalRobotStatus() {
    try {
      const res = await fetch('/api/external_robot/status');
      if (res.ok) {
        const data = await res.json();
        updateExternalRobotUI(data);
      }
    } catch (e) {}
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
  function bootstrap() {
    initThree();
    attachEvents();
    fetchStatusAndRobot();
    initSse();
    checkUrlParams();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', bootstrap);
  } else {
    bootstrap();
  }
})();
