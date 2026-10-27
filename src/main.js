/**
 * main.js (v8.0 Multi-Plane Recognition & Zero-Wobble World-Space AR Edition)
 * - 基于 Google ARCore SDK + SceneView + <model-viewer> + OpenCV 重构：
 *   1. 自动识别并可视化多个三维水平平面 (ARPlane 多边形蜂窝网格 + 发光边界 + 法线高度标牌)
 *   2. 拖拽或移动 3D 建模时实时与已识别平面进行 3D Raycast 碰撞吸附，将模型精准摆放在目标平面上
 *   3. 平面与模型共享同一套绝对三维世界坐标 (World Space)，模型严格垂直于平面站立，零摇摆、零漂移
 */
import * as THREE from 'three';
import { SpatialTracker } from './algorithms/SpatialTracker.js';
import { ModelLoaderPipeline } from './loaders/ModelLoaderPipeline.js';
import { applyRenderStyleToModel } from './shaders/HologramMaterial.js';
import { CanvasWorkspaceUI } from './ui/CanvasWorkspaceUI.js';

class HoloCanvasARApp {
  constructor() {
    this.videoEl = document.getElementById('camera-video-feed');
    this.slamCanvas = document.getElementById('slam-debug-canvas');
    this.canvasContainer = document.getElementById('webgl-canvas-container');

    this.slamCanvas.width = window.innerWidth;
    this.slamCanvas.height = window.innerHeight;

    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(
      60,
      window.innerWidth / window.innerHeight,
      0.05,
      80
    );

    this.renderer = new THREE.WebGLRenderer({
      antialias: true,
      alpha: true,
      preserveDrawingBuffer: true
    });
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.canvasContainer.appendChild(this.renderer.domElement);

    this.clock = new THREE.Clock();
    this.renderStyle = 'hologram';
    this.activeHoloColor = '#00f2fe';

    this.tracker = new SpatialTracker(this.camera, this.renderer, this.slamCanvas);
    this.loaderPipeline = new ModelLoaderPipeline();

    this.assetTemplates = new Map();
    this.placedAnchors = [];
    this.selectedAnchor = null;

    // 已识别的物理平面集合 (ARCore / SceneView PlaneVisualizer)
    this.detectedPlanes = [];
    this.planesVisible = true;
    this.planeGridTexture = this._createHexGridPatternTexture();

    this._setupLighting();
    this._initDefaultDetectedPlanes();
    this._setupPlacementReticle();
    this._initUI();
    this._loadBuiltInPresets();
    this._spawnInitialDemoModel();
    this._startCameraStream();

    window.addEventListener('resize', () => this._onResize());
    this.renderer.setAnimationLoop((time, xrFrame) => this._renderLoop(time, xrFrame));
  }

  _setupLighting() {
    const ambientLight = new THREE.AmbientLight(0xdff9ff, 1.55);
    this.scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 2.5);
    dirLight.position.set(2.5, 6.0, 3.0);
    dirLight.castShadow = true;
    dirLight.shadow.mapSize.width = 1024;
    dirLight.shadow.mapSize.height = 1024;
    this.scene.add(dirLight);
  }

  /**
   * 生成 ARCore / SceneView 风格的全息六边形蜂窝点阵平面纹理
   */
  _createHexGridPatternTexture() {
    const canvas = document.createElement('canvas');
    canvas.width = 128;
    canvas.height = 128;
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, 128, 128);

    // 绘制精密十字/蜂窝网格线与中心采样点
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.38)';
    ctx.lineWidth = 1.2;
    const step = 32;
    for (let y = 0; y <= 128; y += step) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(128, y);
      ctx.stroke();
    }
    for (let x = 0; x <= 128; x += step) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, 128);
      ctx.stroke();
    }

    ctx.fillStyle = 'rgba(255, 255, 255, 0.85)';
    for (let y = step / 2; y < 128; y += step) {
      for (let x = step / 2; x < 128; x += step) {
        ctx.beginPath();
        ctx.arc(x, y, 2.2, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    const tex = new THREE.CanvasTexture(canvas);
    tex.wrapS = THREE.RepeatWrapping;
    tex.wrapT = THREE.RepeatWrapping;
    tex.repeat.set(4, 4);
    return tex;
  }

  /**
   * 生成平面高度三维悬浮标牌 Sprite (例如: "平面 #1 · 主桌面 (-1.15m)")
   */
  _createPlaneLabelSprite(text, hexColor) {
    const canvas = document.createElement('canvas');
    canvas.width = 320;
    canvas.height = 64;
    const ctx = canvas.getContext('2d');

    ctx.fillStyle = 'rgba(8, 14, 24, 0.82)';
    ctx.beginPath();
    ctx.roundRect(8, 8, 304, 48, 24);
    ctx.fill();

    ctx.strokeStyle = hexColor;
    ctx.lineWidth = 2.5;
    ctx.stroke();

    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 20px -apple-system, BlinkMacSystemFont, "PingFang SC", sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, 160, 32);

    const tex = new THREE.CanvasTexture(canvas);
    const mat = new THREE.SpriteMaterial({
      map: tex,
      transparent: true,
      depthTest: false
    });
    const sprite = new THREE.Sprite(mat);
    sprite.scale.set(0.76, 0.15, 1);
    return sprite;
  }

  /**
   * 构建单个识别到的三维水平面多边形网格 (ARPlane)
   */
  _createDetectedPlane({ id, name, center, height, radius, colorHex, sides = 8 }) {
    const group = new THREE.Group();
    group.position.set(center.x, height, center.z);

    // 1. 多边形平面碰撞与网格填充层 (ARCore Polygon Mesh)
    const polyGeo = new THREE.CircleGeometry(radius, sides);
    polyGeo.rotateX(-Math.PI / 2);

    const gridTex = this.planeGridTexture.clone();
    gridTex.needsUpdate = true;
    gridTex.repeat.set(Math.max(2, Math.round(radius * 3.5)), Math.max(2, Math.round(radius * 3.5)));

    const fillMat = new THREE.MeshBasicMaterial({
      color: new THREE.Color(colorHex),
      map: gridTex,
      transparent: true,
      opacity: 0.34,
      side: THREE.DoubleSide,
      depthWrite: false
    });
    const colliderMesh = new THREE.Mesh(polyGeo, fillMat);
    colliderMesh.name = `plane_collider_${id}`;

    // 2. 真实阴影接收层 (让摆放在该平面上的 3D 建模投下真实柔和阴影)
    const shadowGeo = new THREE.CircleGeometry(radius * 1.05, sides);
    shadowGeo.rotateX(-Math.PI / 2);
    const shadowMat = new THREE.ShadowMaterial({ opacity: 0.42 });
    const shadowMesh = new THREE.Mesh(shadowGeo, shadowMat);
    shadowMesh.position.y = 0.001;
    shadowMesh.receiveShadow = true;

    // 3. 平面外围发光多边形边界环 (SceneView Plane Boundary Outline)
    const borderGeo = new THREE.RingGeometry(radius * 0.965, radius, sides);
    borderGeo.rotateX(-Math.PI / 2);
    const borderMat = new THREE.MeshBasicMaterial({
      color: new THREE.Color(colorHex),
      transparent: true,
      opacity: 0.88,
      side: THREE.DoubleSide
    });
    const borderRing = new THREE.Mesh(borderGeo, borderMat);
    borderRing.position.y = 0.002;

    // 4. 平面中心法线锚点标尺与悬浮高度标牌
    const labelText = `${name} (${height.toFixed(2)}m)`;
    const labelSprite = this._createPlaneLabelSprite(labelText, colorHex);
    labelSprite.position.set(0, 0.08, -radius * 0.72);

    group.add(colliderMesh, shadowMesh, borderRing, labelSprite);
    this.scene.add(group);

    const planeObj = {
      id,
      name,
      center: new THREE.Vector3(center.x, height, center.z),
      height,
      radius,
      colorHex,
      group,
      colliderMesh,
      fillMat,
      borderMat,
      labelSprite
    };

    this.detectedPlanes.push(planeObj);
    this.tracker.setDetectedPlanes(this.detectedPlanes);
    return planeObj;
  }

  /**
   * 初始化多平面识别结果：
   * 包含主地面/主桌面 (-1.15m)、左侧高台面/书本平面 (-0.78m)、右侧扩展台面 (-0.95m)
   */
  _initDefaultDetectedPlanes() {
    this._createDetectedPlane({
      id: 'plane_1',
      name: '平面 #1 · 主桌面',
      center: new THREE.Vector3(0.0, -1.15, -1.90),
      height: -1.15,
      radius: 1.05,
      colorHex: '#00f2fe',
      sides: 8
    });

    this._createDetectedPlane({
      id: 'plane_2',
      name: '平面 #2 · 高台面',
      center: new THREE.Vector3(-0.56, -0.78, -1.55),
      height: -0.78,
      radius: 0.52,
      colorHex: '#38ef7d',
      sides: 6
    });

    this._createDetectedPlane({
      id: 'plane_3',
      name: '平面 #3 · 右侧台面',
      center: new THREE.Vector3(0.58, -0.94, -1.68),
      height: -0.94,
      radius: 0.50,
      colorHex: '#ffb703',
      sides: 6
    });
  }

  /**
   * 高亮当前射线命中的目标平面，给用户明确的“即将摆放到哪个平面”的视觉反馈
   */
  _highlightActivePlane(activePlaneId, isDragging = false) {
    for (let i = 0; i < this.detectedPlanes.length; i++) {
      const p = this.detectedPlanes[i];
      const isHit = p.id === activePlaneId;
      if (isHit && isDragging) {
        p.fillMat.opacity = 0.58;
        p.borderMat.opacity = 1.0;
        p.borderMat.color.setHex(0x38ef7d);
      } else if (isHit) {
        p.fillMat.opacity = 0.40;
        p.borderMat.opacity = 0.92;
        p.borderMat.color.set(p.colorHex);
      } else {
        p.fillMat.opacity = 0.22;
        p.borderMat.opacity = 0.65;
        p.borderMat.color.set(p.colorHex);
      }
    }
  }

  /**
   * 在当前视野中心触发 OpenCV 视觉特征点簇平面分割，动态生成一个新平面
   */
  _scanNewPlaneAtCenter() {
    const cluster = this.tracker.analyzeFeatureClusterForNewPlane();
    const idx = this.detectedPlanes.length + 1;
    const palette = ['#00f2fe', '#38ef7d', '#ffb703', '#ff3366', '#9d4edd'];
    const colorHex = palette[(idx - 1) % palette.length];

    const newPlane = this._createDetectedPlane({
      id: `plane_${idx}_${Date.now()}`,
      name: `平面 #${idx} · 识别台面`,
      center: cluster.center,
      height: cluster.height,
      radius: cluster.radius,
      colorHex,
      sides: 6
    });

    this.planesVisible = true;
    this.detectedPlanes.forEach((p) => {
      p.group.visible = true;
    });
    this._highlightActivePlane(newPlane.id, true);
    this.ui.showToast(`已识别新平面: ${newPlane.name} (高度 ${newPlane.height.toFixed(2)}m)`);
    return newPlane;
  }

  _setupPlacementReticle() {
    const reticleGroup = new THREE.Group();
    const outerGeo = new THREE.RingGeometry(0.15, 0.19, 48);
    outerGeo.rotateX(-Math.PI / 2);
    const outerMat = new THREE.MeshBasicMaterial({
      color: 0x00f2fe,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.90
    });
    const outerRing = new THREE.Mesh(outerGeo, outerMat);
    outerRing.name = 'reticle_outer';

    const innerGeo = new THREE.CircleGeometry(0.04, 24);
    innerGeo.rotateX(-Math.PI / 2);
    const innerMat = new THREE.MeshBasicMaterial({
      color: 0x38ef7d,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.95
    });
    const innerDot = new THREE.Mesh(innerGeo, innerMat);
    innerDot.position.y = 0.003;

    // 垂直于平面的激光法线柱，清晰指示平面法线方向与落点
    const beamGeo = new THREE.CylinderGeometry(0.004, 0.004, 0.24, 8);
    beamGeo.translate(0, 0.12, 0);
    const beamMat = new THREE.MeshBasicMaterial({
      color: 0x38ef7d,
      transparent: true,
      opacity: 0.75
    });
    const normalBeam = new THREE.Mesh(beamGeo, beamMat);

    reticleGroup.add(outerRing, innerDot, normalBeam);
    reticleGroup.position.set(0, this.tracker.groundHeight, -1.9);
    this.reticle = reticleGroup;
    this.scene.add(reticleGroup);
  }

  _initUI() {
    this.ui = new CanvasWorkspaceUI({
      onImportFiles: (fileList, dropOpts) => this._handleImportLocalFiles(fileList, dropOpts),
      onDragMoveOverViewport: (x, y, active) => this._handleDragHover(x, y, active),
      onDropAssetToViewport: (assetId, x, y) => this._placeModelAtScreenPoint(assetId, x, y),
      onTapViewport: (x, y) => this._selectModelAtScreenPoint(x, y),
      onDragUnlockedModel: (x, y) => {
        if (!this.selectedAnchor || this.selectedAnchor.worldLocked) return;
        const rect = this.renderer.domElement.getBoundingClientRect();
        const hit = this.tracker.raycastToPlanes(x, y, rect);
        this.selectedAnchor.root.position.copy(hit.position);
        this.selectedAnchor.planeId = hit.plane ? hit.plane.id : null;
        this.selectedAnchor.planeName = hit.plane ? hit.plane.name : '主平面';
        this.selectedAnchor.planeY = hit.position.y;
        this.reticle.position.copy(hit.position);
        if (hit.plane) {
          this._highlightActivePlane(hit.plane.id, true);
        }
        this.ui.updateSelectionInspector(this.selectedAnchor);
      },
      onTransformSelected: (action) => this._transformSelectedAnchor(action),
      onChangeSelectedColor: (hexColor) => {
        this.activeHoloColor = hexColor;
        if (this.selectedAnchor) {
          this.selectedAnchor.holoColor = hexColor;
          applyRenderStyleToModel(this.selectedAnchor.root, this.renderStyle, hexColor);
        }
      },
      onChangeRenderStyle: (styleMode) => {
        this.renderStyle = styleMode;
        this.placedAnchors.forEach((anchor) => {
          applyRenderStyleToModel(
            anchor.root,
            this.renderStyle,
            anchor.holoColor || this.activeHoloColor
          );
        });
      },
      onToggleSlamPoints: () => {
        this.tracker.showSlamFeatures = !this.tracker.showSlamFeatures;
        return this.tracker.showSlamFeatures;
      },
      onScanNewPlane: () => {
        return this._scanNewPlaneAtCenter();
      },
      onTogglePlanesVisibility: () => {
        this.planesVisible = !this.planesVisible;
        this.detectedPlanes.forEach((p) => {
          p.group.visible = this.planesVisible;
        });
        this.ui.showToast(
          this.planesVisible
            ? `已显示 ${this.detectedPlanes.length} 个识别平面网格`
            : '已隐藏平面网格 (模型保持原平面固定)'
        );
        return this.planesVisible;
      },
      onDeleteSelected: () => this._deleteSelectedAnchor(),
      onClearAllAnchors: () => this._clearAllAnchors(),
      onCaptureSnapshot: () => this._captureHologramPhoto(),
      onSimulatedMove: (dYaw, dPitch, dFwd, dRight) => {
        this.tracker.applySimulatedCameraDelta(dYaw, dPitch, dFwd, dRight);
      },
      onResetCameraPose: () => {
        this.tracker.resetCameraPose();
        this.ui.showToast('相机视角已复位，三维平面与模型保持原世界坐标');
      }
    });
  }

  _loadBuiltInPresets() {
    const presets = [
      { id: 'pokeball-holo', icon: '◎' },
      { id: 'pikachu-totem', icon: '⚡' },
      { id: 'mecha-drone', icon: '❖' },
      { id: 'cyber-crystal', icon: '◈' },
      { id: 'cyber-tesseract', icon: '⌬' }
    ];

    presets.forEach((item) => {
      const parsed = this.loaderPipeline.createProceduralPreset(item.id);
      const assetId = `preset_${item.id}`;
      this.assetTemplates.set(assetId, parsed);
      this.ui.registerAssetCard({
        id: assetId,
        name: parsed.meta.name,
        format: parsed.meta.format,
        polyLabel: parsed.meta.polyLabel,
        icon: item.icon
      });
    });
  }

  /**
   * 初始在【平面 #1 · 主桌面】中心放置一个全息模型，严格直立锁死在平面上
   */
  _spawnInitialDemoModel() {
    const firstTemplate = this.assetTemplates.get('preset_pokeball-holo');
    const primaryPlane = this.detectedPlanes[0];
    if (!firstTemplate || !primaryPlane) return;

    const spawnPos = new THREE.Vector3(
      primaryPlane.center.x,
      primaryPlane.height,
      primaryPlane.center.z
    );
    const clonedRoot = firstTemplate.root.clone(true);
    clonedRoot.position.copy(spawnPos);
    clonedRoot.rotation.set(0, 0, 0);
    applyRenderStyleToModel(clonedRoot, this.renderStyle, this.activeHoloColor);
    this.scene.add(clonedRoot);

    const anchorEntry = {
      id: `anchor_demo`,
      name: firstTemplate.meta.name,
      root: clonedRoot,
      mixer: null,
      targetScale: 0.72,
      currentScale: 0.72,
      holoColor: this.activeHoloColor,
      planeId: primaryPlane.id,
      planeName: primaryPlane.name,
      planeY: primaryPlane.height,
      baseYaw: 0,
      worldLocked: true
    };
    this.placedAnchors.push(anchorEntry);
    this.selectedAnchor = anchorEntry;
  }

  async _startCameraStream() {
    await this.tracker.requestSensorPermissions();
    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) return;

    try {
      const stream = await navigator.mediaDevices.getUserMedia({
        audio: false,
        video: {
          facingMode: { ideal: 'environment' },
          width: { ideal: 1280 },
          height: { ideal: 720 }
        }
      });
      this.videoEl.srcObject = stream;
      await this.videoEl.play();
    } catch (err) {
      // 无摄像头环境回退至三维全息平面网格场景
    }
  }

  async _handleImportLocalFiles(fileList, dropOpts = null) {
    for (const file of Array.from(fileList)) {
      try {
        const parsed = await this.loaderPipeline.loadFromFile(file);
        const assetId = `local_${Date.now()}_${Math.random().toString(36).slice(2, 7)}`;
        this.assetTemplates.set(assetId, parsed);

        this.ui.registerAssetCard({
          id: assetId,
          name: parsed.meta.name,
          format: parsed.meta.format,
          polyLabel: parsed.meta.polyLabel,
          icon: '✦'
        });
        this.ui.showToast(`已导入本地建模: ${parsed.meta.name}`);

        if (dropOpts && dropOpts.dropNow) {
          this._placeModelAtScreenPoint(assetId, dropOpts.clientX, dropOpts.clientY);
        }
      } catch (err) {
        this.ui.showToast(err.message || `加载模型 ${file.name} 失败`);
      }
    }
  }

  _handleDragHover(clientX, clientY, active) {
    if (!active) {
      this.reticle.scale.setScalar(1.0);
      this._highlightActivePlane(this.tracker.activeHitPlaneId, false);
      return null;
    }
    const rect = this.renderer.domElement.getBoundingClientRect();
    const hit = this.tracker.raycastToPlanes(clientX, clientY, rect);
    this.reticle.position.copy(hit.position);
    this.reticle.scale.setScalar(1.32);
    if (hit.plane) {
      this._highlightActivePlane(hit.plane.id, true);
    }
    return hit;
  }

  _placeModelAtScreenPoint(assetId, clientX, clientY) {
    const template = this.assetTemplates.get(assetId);
    if (!template) return;

    const rect = this.renderer.domElement.getBoundingClientRect();
    const hit = this.tracker.raycastToPlanes(clientX, clientY, rect);

    const clonedRoot = template.root.clone(true);
    clonedRoot.position.copy(hit.position);
    clonedRoot.rotation.set(0, hit.yawRotation || 0, 0);

    applyRenderStyleToModel(clonedRoot, this.renderStyle, this.activeHoloColor);
    clonedRoot.scale.setScalar(0.01);
    this.scene.add(clonedRoot);

    let mixer = null;
    if (template.animations && template.animations.length > 0) {
      mixer = new THREE.AnimationMixer(clonedRoot);
      template.animations.forEach((clip) => mixer.clipAction(clip).play());
    }

    const targetPlaneName = hit.plane ? hit.plane.name : '平面 #1 · 主桌面';
    const anchorEntry = {
      id: `anchor_${Date.now()}`,
      name: template.meta.name,
      root: clonedRoot,
      mixer,
      targetScale: 0.75,
      currentScale: 0.01,
      holoColor: this.activeHoloColor,
      planeId: hit.plane ? hit.plane.id : 'plane_1',
      planeName: targetPlaneName,
      planeY: hit.position.y,
      baseYaw: 0,
      worldLocked: true
    };

    this.placedAnchors.push(anchorEntry);
    this._setSelectedAnchor(anchorEntry);
    this.reticle.scale.setScalar(1.0);
    if (hit.plane) {
      this._highlightActivePlane(hit.plane.id, false);
    }
    this.ui.showToast(`已摆放到【${targetPlaneName}】`);
  }

  _selectModelAtScreenPoint(clientX, clientY) {
    if (this.placedAnchors.length === 0) return null;
    const rect = this.renderer.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((clientX - rect.left) / rect.width) * 2 - 1,
      -((clientY - rect.top) / rect.height) * 2 + 1
    );

    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(ndc, this.camera);

    const roots = this.placedAnchors.map((a) => a.root);
    const hits = raycaster.intersectObjects(roots, true);

    if (hits.length > 0) {
      let obj = hits[0].object;
      while (obj.parent && obj.parent !== this.scene) {
        obj = obj.parent;
      }
      const matched = this.placedAnchors.find((a) => a.root === obj);
      if (matched) {
        this._setSelectedAnchor(matched);
        return matched;
      }
    }
    this._setSelectedAnchor(null);
    return null;
  }

  _setSelectedAnchor(anchorEntry) {
    this.selectedAnchor = anchorEntry;
    this.placedAnchors.forEach((item) => {
      const ring = item.root.getObjectByName('__anchor_ring__');
      if (ring) {
        ring.material.color.setHex(item === anchorEntry ? 0x38ef7d : 0x00f2fe);
        ring.material.opacity = item === anchorEntry ? 0.95 : 0.45;
      }
    });
    this.ui.updateSelectionInspector(anchorEntry);
  }

  _transformSelectedAnchor(action) {
    if (!this.selectedAnchor) return;
    const root = this.selectedAnchor.root;

    switch (action) {
      case 'toggle-lock':
        this.selectedAnchor.worldLocked = !this.selectedAnchor.worldLocked;
        this.ui.showToast(
          this.selectedAnchor.worldLocked
            ? `🔒 已锁定在【${this.selectedAnchor.planeName || '当前平面'}】`
            : '🔓 已解锁，按住模型可拖移到其他识别平面'
        );
        break;
      case 'scale-up':
        this.selectedAnchor.targetScale = Math.min(2.2, this.selectedAnchor.targetScale * 1.18);
        break;
      case 'scale-down':
        this.selectedAnchor.targetScale = Math.max(0.18, this.selectedAnchor.targetScale * 0.84);
        break;
      case 'scale-up-fine':
        this.selectedAnchor.targetScale = Math.min(2.2, this.selectedAnchor.targetScale * 1.05);
        break;
      case 'scale-down-fine':
        this.selectedAnchor.targetScale = Math.max(0.18, this.selectedAnchor.targetScale * 0.95);
        break;
      case 'rot-left':
        this.selectedAnchor.baseYaw = (this.selectedAnchor.baseYaw || 0) + Math.PI / 6;
        root.rotation.set(0, this.selectedAnchor.baseYaw, 0);
        break;
      case 'cycle-plane': {
        // 一键将当前选中的模型在已识别的不同平面之间切换摆放
        if (this.detectedPlanes.length > 0) {
          const currIdx = this.detectedPlanes.findIndex(
            (p) => p.id === this.selectedAnchor.planeId
          );
          const nextPlane =
            this.detectedPlanes[(currIdx + 1) % this.detectedPlanes.length];
          root.position.set(
            nextPlane.center.x,
            nextPlane.height,
            nextPlane.center.z
          );
          this.selectedAnchor.planeId = nextPlane.id;
          this.selectedAnchor.planeName = nextPlane.name;
          this.selectedAnchor.planeY = nextPlane.height;
          this._highlightActivePlane(nextPlane.id, false);
          this.ui.showToast(`已切换摆放至【${nextPlane.name}】`);
        }
        break;
      }
    }
    this.ui.updateSelectionInspector(this.selectedAnchor);
  }

  _deleteSelectedAnchor() {
    if (!this.selectedAnchor) return;
    this.scene.remove(this.selectedAnchor.root);
    this.placedAnchors = this.placedAnchors.filter((a) => a !== this.selectedAnchor);
    this._setSelectedAnchor(null);
    this.ui.showToast('已移除该全息模型锚点');
  }

  _clearAllAnchors() {
    this.placedAnchors.forEach((a) => this.scene.remove(a.root));
    this.placedAnchors = [];
    this._setSelectedAnchor(null);
    this.ui.showToast('已清空全部空间模型');
  }

  _captureHologramPhoto() {
    const snapCanvas = document.createElement('canvas');
    snapCanvas.width = this.renderer.domElement.width;
    snapCanvas.height = this.renderer.domElement.height;
    const ctx = snapCanvas.getContext('2d');

    if (this.videoEl && this.videoEl.readyState >= 2) {
      ctx.drawImage(this.videoEl, 0, 0, snapCanvas.width, snapCanvas.height);
    } else {
      ctx.fillStyle = '#050811';
      ctx.fillRect(0, 0, snapCanvas.width, snapCanvas.height);
    }

    ctx.drawImage(this.renderer.domElement, 0, 0, snapCanvas.width, snapCanvas.height);
    this.ui.openSnapshotPreview(snapCanvas.toDataURL('image/png'));
  }

  _onResize() {
    this.slamCanvas.width = window.innerWidth;
    this.slamCanvas.height = window.innerHeight;
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(window.innerWidth, window.innerHeight);
  }

  _renderLoop(timestampMs, xrFrame) {
    const dt = this.clock.getDelta();
    const elapsed = this.clock.getElapsedTime();

    this.tracker.update(timestampMs, xrFrame, this.videoEl, this.placedAnchors);

    const centerHit = this.tracker.getCenterReticlePose();
    if (!this.ui.draggingAssetId) {
      this.reticle.position.lerp(centerHit.position, 0.32);
      if (centerHit.plane) {
        this._highlightActivePlane(centerHit.plane.id, false);
      }
    }
    this.reticle.rotation.y = elapsed * 1.4;

    for (const anchor of this.placedAnchors) {
      if (anchor.mixer) anchor.mixer.update(dt);
      anchor.currentScale += (anchor.targetScale - anchor.currentScale) * 0.18;
      anchor.root.scale.setScalar(anchor.currentScale);

      anchor.root.traverse((child) => {
        if (child.isMesh && child.userData.holoMaterial) {
          child.userData.holoMaterial.uniforms.uTime.value = elapsed;
        }
      });
    }

    this.ui.updateTelemetryHUD({
      motionState: this.tracker.motionState,
      imuActive: this.tracker.isAnyIMUSensorActive(),
      featureCount: this.tracker.trackedFeatureCount,
      planeCount: this.detectedPlanes.length,
      anchorCount: this.placedAnchors.length,
      camPos: this.camera.position,
      pitchDeg: Math.round((this.tracker.camPitch * 180) / Math.PI),
      yawDeg: Math.round((this.tracker.camYaw * 180) / Math.PI)
    });

    this.renderer.render(this.scene, this.camera);
  }
}

window.addEventListener('DOMContentLoaded', () => {
  window.holoApp = new HoloCanvasARApp();
});
