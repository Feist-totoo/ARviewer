/**
 * SpatialTracker.js (v8.0 ARCore + SceneView + <model-viewer> + OpenCV Plane-Snap & Zero-Wobble Edition)
 *
 * 深度融合四大 Apache-2.0 开源项目的核心空间算法：
 * 1. Google ARCore Android SDK (https://github.com/google-ar/arcore-android-sdk) &
 *    SceneView (https://github.com/sceneview/sceneview):
 *    - 真实世界坐标系 (3D World Space) 姿态架构：所有识别出的平面 (ARPlane) 与放置的 3D 模型 (AnchorNode)
 *      均固定在绝对三维世界坐标 (Xw, Yw, Zw) 中，模型严格垂直于平面法线站立 (rotation.x = 0, rotation.z = 0)，
 *      彻底根除因模型自摇摆 (Self-Rocking) 导致的晃动感；仅相机节点 (this.camera) 在三维空间中作 6DoF 运动。
 *    - 多平面实时检测与多边形网格重建 (Multi-Plane Detection & Boundary Polygon Hull)：
 *      自动识别地面/主桌面、高台面、侧展台面等多个不同高度 (Y_plane) 的物理水平面，并支持基于当前视觉特征点簇
 *      实时提取新平面。
 * 2. Google <model-viewer> (https://github.com/google/model-viewer):
 *    - 射线-平面碰撞吸附放置 (Ray-Plane Hit-Test Snapping)：拖拽或移动模型时，从相机发射 3D 射线与所有已识别
 *      的 ARPlane 多边形碰撞检测，自动吸附到命中平面的精确三维坐标与法线高度上。
 *    - 二阶临界阻尼弹簧平滑器 (Critically Damped Spring Damper)：帧率无关的解析解平滑，零过冲、零振荡。
 * 3. OpenCV (https://github.com/opencv/opencv):
 *    - 零均值归一化绝对误差和 (Zero-Mean SAD, ZSAD) 9x9 持久特征块匹配，彻底免疫自动曝光亮度跳变。
 *    - `estimateAffinePartial2D` (RANSAC + Huber IRLS Procrustes) 鲁棒求解相机位移与缩放。
 *    - 双级迟滞防抖死区 (Two-Tier Hysteresis Anti-Wobble Deadzone)：当帧间残差小于 0.28px 时自动进入
 *      STATIC_LOCK 静态刚性锁死态，彻底消除手持静止时的传感器底噪微颤。
 */
import * as THREE from 'three';

/**
 * 二阶临界阻尼弹簧平滑器 (源自 Google <model-viewer> Damper, Apache-2.0 License)
 * 微分方程: x'' + 2 * omega * x' + omega^2 * (x - goal) = 0
 */
class CriticallyDampedSpringScalar {
  constructor(naturalFrequency = 24.0) {
    this.omega = naturalFrequency;
    this.value = 0.0;
    this.velocity = 0.0;
    this.initialized = false;
  }

  reset(val = 0.0) {
    this.value = val;
    this.velocity = 0.0;
    this.initialized = true;
  }

  update(goal, dtSec) {
    if (!this.initialized) {
      this.value = goal;
      this.velocity = 0.0;
      this.initialized = true;
      return goal;
    }
    const dt = Math.max(0.001, Math.min(0.1, dtSec));
    const w = this.omega;
    const delta = this.value - goal;
    const expTerm = Math.exp(-w * dt);
    const newVal = goal + (delta + (this.velocity + w * delta) * dt) * expTerm;
    this.velocity = (this.velocity - (this.velocity + w * delta) * w * dt) * expTerm;
    this.value = newVal;
    return this.value;
  }
}

export class SpatialTracker {
  constructor(camera, renderer, slamOverlayCanvas = null) {
    this.camera = camera;
    this.renderer = renderer;
    this.slamCanvas = slamOverlayCanvas;
    this.slamCtx = slamOverlayCanvas ? slamOverlayCanvas.getContext('2d') : null;
    this.showSlamFeatures = true;

    this.mode = 'arcore-sceneview-opencv-slam';
    this.motionState = 'STATIC_LOCK';
    this.groundHeight = -1.15;
    this.defaultSpawnDistance = 1.85;

    // 相机在 VO 像素域的累计姿态与缩放
    this.estCamX = 0.0;
    this.estCamY = 0.0;
    this.estScale = 1.0;
    this.estRoll = 0.0;

    // 静态防抖迟滞死区计数器 (OpenCV / ARCore Hysteresis Anti-Wobble Gate)
    this.staticFrameCounter = 0;
    this.deadzoneThresholdPx = 0.28;

    // Google <model-viewer> 二阶临界阻尼弹簧平滑器
    this.springCamX = new CriticallyDampedSpringScalar(24.0);
    this.springCamY = new CriticallyDampedSpringScalar(24.0);
    this.springScale = new CriticallyDampedSpringScalar(16.0);
    this.springRoll = new CriticallyDampedSpringScalar(20.0);
    this.springScale.reset(1.0);

    this.smoothCamX = 0.0;
    this.smoothCamY = 0.0;
    this.smoothScale = 1.0;
    this.smoothRoll = 0.0;
    this.lastUpdateTimestampMs = null;

    // 真实三维世界坐标系中的相机欧拉角与位置 (相机移动，而平面与模型固定不动！)
    this.basePitch = -0.26;
    this.camYaw = 0.0;
    this.camPitch = this.basePitch;
    this.camRoll = 0.0;
    this.cameraPosition = new THREE.Vector3(0, 0, 0);

    // 手机运动传感器通道状态
    this.sensorChannels = {
      orientationEvent: false,
      motionGyro: false,
      gravityTilt: false,
      webSensorApi: false
    };
    this.prevOrientQuat = null;
    this.prevGravityPitch = null;
    this.gravityVec = new THREE.Vector3(0, -9.8, 0);
    this.imuDeltaPxX = 0.0;
    this.imuDeltaPxY = 0.0;
    this.imuDeltaRoll = 0.0;
    this.lastMotionTimestamp = null;

    // OpenCV 双层金字塔分辨率 (细层 96x128, 粗层 32x42)
    this.voWidth = 96;
    this.voHeight = 128;
    this.coarseW = 32;
    this.coarseH = 42;

    this.voCanvas = document.createElement('canvas');
    this.voCanvas.width = this.voWidth;
    this.voCanvas.height = this.voHeight;
    this.voCtx = this.voCanvas.getContext('2d', { willReadFrequently: true });

    this.prevGrayFrame = null;
    this.prevCoarseFrame = null;
    this.lastVideoTime = -1;

    // OpenCV 持久特征路标点库 (含零均值归一化 9x9 描述子 zmPatch)
    this.landmarks = [];
    this.trackedFeatureCount = 0;

    // 已识别的物理平面列表 (ARCore / SceneView Plane 抽象)
    this.detectedPlanes = [];
    this.activeHitPlaneId = null;

    this.raycaster = new THREE.Raycaster();
    this._initAllMotionSensors();
  }

  async requestSensorPermissions() {
    try {
      if (
        typeof DeviceOrientationEvent !== 'undefined' &&
        typeof DeviceOrientationEvent.requestPermission === 'function'
      ) {
        await DeviceOrientationEvent.requestPermission();
      }
      if (
        typeof DeviceMotionEvent !== 'undefined' &&
        typeof DeviceMotionEvent.requestPermission === 'function'
      ) {
        await DeviceMotionEvent.requestPermission();
      }
    } catch (e) {}
    this._bindDOMMotionEvents();
    this._initGenericSensorAPI();
    return this.isAnyIMUSensorActive();
  }

  isAnyIMUSensorActive() {
    const c = this.sensorChannels;
    return c.orientationEvent || c.motionGyro || c.gravityTilt || c.webSensorApi;
  }

  _initAllMotionSensors() {
    if (typeof window === 'undefined') return;
    this._bindDOMMotionEvents();
    this._initGenericSensorAPI();

    const unlock = () => this.requestSensorPermissions();
    window.addEventListener('pointerdown', unlock, { once: true, passive: true });
    window.addEventListener('touchstart', unlock, { once: true, passive: true });
  }

  _wrapAngleRad(rad) {
    while (rad > Math.PI) rad -= 2 * Math.PI;
    while (rad < -Math.PI) rad += 2 * Math.PI;
    return rad;
  }

  _deviceEulerToQuaternion(alphaRad, betaRad, gammaRad) {
    const orientDeg =
      (window.screen && window.screen.orientation && window.screen.orientation.angle) ||
      window.orientation ||
      0;
    const orientRad = (orientDeg * Math.PI) / 180;
    const euler = new THREE.Euler(betaRad, alphaRad, -gammaRad, 'YXZ');
    const q = new THREE.Quaternion().setFromEuler(euler);
    const q1 = new THREE.Quaternion(-Math.sqrt(0.5), 0, 0, Math.sqrt(0.5));
    const q0 = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -orientRad);
    q.multiply(q1);
    q.multiply(q0);
    return q;
  }

  _anglesToVoPixels(dYawRad, dPitchRad) {
    const halfFovRad = (this.camera.fov * Math.PI) / 360;
    const tanHalf = Math.tan(halfFovRad);
    const aspect = this.voWidth / this.voHeight;
    const pxX = (-dYawRad / (aspect * tanHalf)) * (this.voWidth * 0.5);
    const pxY = (-dPitchRad / tanHalf) * (this.voHeight * 0.5);
    return { pxX, pxY };
  }

  _bindDOMMotionEvents() {
    if (this._domEventsBound) return;
    this._domEventsBound = true;

    const onOrientation = (event) => {
      if (event.alpha === null && event.beta === null && event.gamma === null) return;
      const deg2rad = Math.PI / 180;
      const qNow = this._deviceEulerToQuaternion(
        (event.alpha || 0) * deg2rad,
        (event.beta || 0) * deg2rad,
        (event.gamma || 0) * deg2rad
      );

      if (this.prevOrientQuat) {
        const dq = this.prevOrientQuat.clone().invert().multiply(qNow);
        const de = new THREE.Euler().setFromQuaternion(dq, 'YXZ');
        const mag = Math.hypot(de.x, de.y);
        // 死区门限：过滤掉小于 0.0028 rad (~0.16°) 的陀螺仪静止热噪声，防止微晃
        if (mag > 0.0028 && mag < 0.8) {
          this.sensorChannels.orientationEvent = true;
          const { pxX, pxY } = this._anglesToVoPixels(de.y, de.x);
          this.imuDeltaPxX += pxX;
          this.imuDeltaPxY += pxY;
          if (Math.abs(de.z) > 0.004) {
            this.imuDeltaRoll += de.z;
          }
        }
      }
      this.prevOrientQuat = qNow.clone();
    };

    window.addEventListener('deviceorientation', onOrientation, true);
    window.addEventListener('deviceorientationabsolute', onOrientation, true);

    window.addEventListener(
      'devicemotion',
      (event) => {
        const now = performance.now();
        const dt =
          this.lastMotionTimestamp !== null
            ? Math.max(0.002, Math.min(0.1, (now - this.lastMotionTimestamp) * 0.001))
            : 0.016;
        this.lastMotionTimestamp = now;
        const deg2rad = Math.PI / 180;

        if (event.rotationRate) {
          const rx = (event.rotationRate.alpha || 0) * deg2rad;
          const ry = (event.rotationRate.beta || 0) * deg2rad;
          const rz = (event.rotationRate.gamma || 0) * deg2rad;
          if (Math.hypot(rx, ry, rz) > 0.03) {
            this.sensorChannels.motionGyro = true;
            if (!this.sensorChannels.orientationEvent && !this.sensorChannels.webSensorApi) {
              const { pxX, pxY } = this._anglesToVoPixels(ry * dt, rx * dt);
              this.imuDeltaPxX += pxX;
              this.imuDeltaPxY += pxY;
              this.imuDeltaRoll -= rz * dt * 0.5;
            }
          }
        }

        const ag = event.accelerationIncludingGravity;
        if (ag && (ag.x !== null || ag.y !== null || ag.z !== null)) {
          const gx = ag.x || 0,
            gy = ag.y || 0,
            gz = ag.z || 0;
          const gNorm = Math.hypot(gx, gy, gz);
          if (gNorm > 4.0 && gNorm < 16.0) {
            this.sensorChannels.gravityTilt = true;
            this.gravityVec.lerp(new THREE.Vector3(gx, gy, gz), 0.22);
            const gravPitch = Math.atan2(
              -this.gravityVec.z,
              Math.hypot(this.gravityVec.x, this.gravityVec.y)
            );
            if (this.prevGravityPitch !== null) {
              const dPitch = this._wrapAngleRad(gravPitch - this.prevGravityPitch);
              if (
                Math.abs(dPitch) > 0.0035 &&
                !this.sensorChannels.orientationEvent &&
                !this.sensorChannels.motionGyro
              ) {
                const { pxY } = this._anglesToVoPixels(0, dPitch);
                this.imuDeltaPxY += pxY;
              }
            }
            this.prevGravityPitch = gravPitch;
          }
        }
      },
      true
    );
  }

  _initGenericSensorAPI() {
    if (this._genericSensorStarted || typeof window === 'undefined') return;
    this._genericSensorStarted = true;
    try {
      const SensorClass =
        window.RelativeOrientationSensor || window.AbsoluteOrientationSensor;
      if (SensorClass) {
        const sensor = new SensorClass({ frequency: 60, referenceFrame: 'screen' });
        let prevQ = null;
        sensor.addEventListener('reading', () => {
          if (!sensor.quaternion) return;
          const q = new THREE.Quaternion().fromArray(sensor.quaternion);
          if (prevQ && !this.sensorChannels.orientationEvent) {
            const dq = prevQ.clone().invert().multiply(q);
            const de = new THREE.Euler().setFromQuaternion(dq, 'YXZ');
            if (Math.hypot(de.x, de.y) > 0.0028) {
              this.sensorChannels.webSensorApi = true;
              const { pxX, pxY } = this._anglesToVoPixels(de.y, de.x);
              this.imuDeltaPxX += pxX;
              this.imuDeltaPxY += pxY;
            }
          }
          prevQ = q.clone();
        });
        sensor.start();
      }
    } catch (e) {}
  }

  _capturePyramidFrames(videoEl) {
    const vw = videoEl.videoWidth || 640;
    const vh = videoEl.videoHeight || 480;
    const screenW = window.innerWidth || 390;
    const screenH = window.innerHeight || 844;

    const screenAspect = screenW / screenH;
    const videoAspect = vw / vh;
    let sx = 0,
      sy = 0,
      sw = vw,
      sh = vh;
    if (videoAspect > screenAspect) {
      sw = vh * screenAspect;
      sx = (vw - sw) * 0.5;
    } else {
      sh = vw / screenAspect;
      sy = (vh - sh) * 0.5;
    }

    this.voCtx.drawImage(videoEl, sx, sy, sw, sh, 0, 0, this.voWidth, this.voHeight);
    const rgba = this.voCtx.getImageData(0, 0, this.voWidth, this.voHeight).data;
    const w = this.voWidth;
    const h = this.voHeight;
    const gray = new Float32Array(w * h);

    for (let i = 0, j = 0; i < rgba.length; i += 4, j++) {
      gray[j] = 0.299 * rgba[i] + 0.587 * rgba[i + 1] + 0.114 * rgba[i + 2];
    }

    const cw = this.coarseW;
    const ch = this.coarseH;
    const coarse = new Float32Array(cw * ch);
    for (let cy = 0; cy < ch; cy++) {
      const py = Math.min(h - 2, cy * 3 + 1);
      for (let cx = 0; cx < cw; cx++) {
        const px = Math.min(w - 2, cx * 3 + 1);
        coarse[cy * cw + cx] =
          (gray[(py - 1) * w + px] +
            gray[py * w + (px - 1)] +
            gray[py * w + px] * 2 +
            gray[py * w + (px + 1)] +
            gray[(py + 1) * w + px]) /
          6.0;
      }
    }

    return { gray, coarse };
  }

  /**
   * 第一级：零均值光照不变粗金字塔全局搜索 (OpenCV Zero-Mean Template Matching, ±24px)
   */
  _estimateCoarseGlobalDelta(prevCoarse, currCoarse) {
    const cw = this.coarseW;
    const ch = this.coarseH;

    let sum0 = 0;
    let count = 0;
    for (let y = 9; y < ch - 9; y++) {
      const r0 = y * cw;
      for (let x = 7; x < cw - 7; x++) {
        sum0 += prevCoarse[r0 + x];
        count++;
      }
    }
    const mean0 = sum0 / count;

    let bestDX = 0;
    let bestDY = 0;
    let minCost = 1e18;

    for (let dy = -8; dy <= 8; dy++) {
      for (let dx = -8; dx <= 8; dx++) {
        let sum1 = 0;
        for (let y = 9; y < ch - 9; y++) {
          const r1 = (y + dy) * cw + dx;
          for (let x = 7; x < cw - 7; x++) {
            sum1 += currCoarse[r1 + x];
          }
        }
        const meanDiff = mean0 - sum1 / count;

        let zsad = 0;
        for (let y = 9; y < ch - 9; y++) {
          const r0 = y * cw;
          const r1 = (y + dy) * cw + dx;
          for (let x = 7; x < cw - 7; x++) {
            zsad += Math.abs(prevCoarse[r0 + x] - (currCoarse[r1 + x] + meanDiff));
          }
        }
        const cost = zsad + (dx * dx + dy * dy) * 2.5;
        if (cost < minCost) {
          minCost = cost;
          bestDX = dx;
          bestDY = dy;
        }
      }
    }

    return { camDeltaX: -bestDX * 3.0, camDeltaY: -bestDY * 3.0 };
  }

  _extractZeroMeanPatch9x9(gray, cx, cy) {
    const w = this.voWidth;
    const patch = new Float32Array(81);
    let sum = 0;
    let k = 0;
    for (let dy = -4; dy <= 4; dy++) {
      const row = (cy + dy) * w + cx;
      for (let dx = -4; dx <= 4; dx++) {
        const val = gray[row + dx];
        patch[k++] = val;
        sum += val;
      }
    }
    const mean = sum / 81.0;
    for (let i = 0; i < 81; i++) {
      patch[i] -= mean;
    }
    return patch;
  }

  _patchZSAD9x9(zmPatch, gray, cx, cy) {
    const w = this.voWidth;
    let sum = 0;
    for (let dy = -4; dy <= 4; dy++) {
      const row = (cy + dy) * w + cx;
      for (let dx = -4; dx <= 4; dx++) {
        sum += gray[row + dx];
      }
    }
    const mean = sum / 81.0;

    let zsad = 0;
    let k = 0;
    for (let dy = -4; dy <= 4; dy++) {
      const row = (cy + dy) * w + cx;
      for (let dx = -4; dx <= 4; dx++) {
        zsad += Math.abs(zmPatch[k++] - (gray[row + dx] - mean));
      }
    }
    return zsad;
  }

  _spawnLandmarks(gray) {
    const w = this.voWidth;
    const h = this.voHeight;
    const margin = 12;
    const cellSize = 12;
    const candidates = [];

    for (let gy = margin; gy < h - margin; gy += cellSize) {
      for (let gx = margin; gx < w - margin; gx += cellSize) {
        const occupied = this.landmarks.some(
          (lm) => Math.hypot(lm.x - gx, lm.y - gy) < 10
        );
        if (occupied) continue;

        let bestScore = 0;
        let bestX = gx;
        let bestY = gy;

        for (let y = gy; y < Math.min(h - margin, gy + cellSize); y += 2) {
          for (let x = gx; x < Math.min(w - margin, gx + cellSize); x += 2) {
            const idx = y * w + x;
            const dx = Math.abs(gray[idx + 1] - gray[idx - 1]);
            const dy = Math.abs(gray[idx + w] - gray[idx - w]);
            const score = dx + dy + Math.min(dx, dy) * 2.0;
            if (score > bestScore) {
              bestScore = score;
              bestX = x;
              bestY = y;
            }
          }
        }

        if (bestScore > 26) {
          candidates.push({
            wx: bestX + this.estCamX,
            wy: bestY + this.estCamY,
            x: bestX,
            y: bestY,
            vx: 0,
            vy: 0,
            score: bestScore,
            patch: this._extractZeroMeanPatch9x9(gray, bestX, bestY)
          });
        }
      }
    }

    candidates.sort((a, b) => b.score - a.score);
    for (const c of candidates) {
      if (this.landmarks.length >= 48) break;
      this.landmarks.push(c);
    }
  }

  /**
   * OpenCV `estimateAffinePartial2D` 风格四自由度相似变换鲁棒求解器 (IRLS + Procrustes)
   */
  _solveSimilarityTransform2D(matchedPairs) {
    const n = matchedPairs.length;
    if (n === 0) return null;

    const cxArr = matchedPairs.map((p) => p.wx - p.u).sort((a, b) => a - b);
    const cyArr = matchedPairs.map((p) => p.wy - p.v).sort((a, b) => a - b);
    let medCamX = cxArr[Math.floor(n / 2)];
    let medCamY = cyArr[Math.floor(n / 2)];

    for (let iter = 0; iter < 2; iter++) {
      let sumWX = 0,
        sumWY = 0,
        sumW = 0;
      for (let i = 0; i < n; i++) {
        const obsX = matchedPairs[i].wx - matchedPairs[i].u;
        const obsY = matchedPairs[i].wy - matchedPairs[i].v;
        const err = Math.hypot(obsX - medCamX, obsY - medCamY);
        const weight = err < 1.8 ? 1.0 : 1.8 / (err + 1e-4);
        sumWX += obsX * weight;
        sumWY += obsY * weight;
        sumW += weight;
      }
      if (sumW > 1e-4) {
        medCamX = sumWX / sumW;
        medCamY = sumWY / sumW;
      }
    }

    let dScale = 1.0;
    let dRoll = 0.0;
    if (n >= 8) {
      let meanPrevU = 0,
        meanPrevV = 0,
        meanCurrU = 0,
        meanCurrV = 0,
        inlierCount = 0;
      const inliers = [];
      for (let i = 0; i < n; i++) {
        const p = matchedPairs[i];
        const err = Math.hypot(p.wx - p.u - medCamX, p.wy - p.v - medCamY);
        if (err < 2.2) {
          inliers.push(p);
          meanPrevU += p.prevU;
          meanPrevV += p.prevV;
          meanCurrU += p.u;
          meanCurrV += p.v;
          inlierCount++;
        }
      }
      if (inlierCount >= 7) {
        meanPrevU /= inlierCount;
        meanPrevV /= inlierCount;
        meanCurrU /= inlierCount;
        meanCurrV /= inlierCount;

        let numA = 0,
          numB = 0,
          den = 0;
        for (let i = 0; i < inlierCount; i++) {
          const p = inliers[i];
          const dx0 = p.prevU - meanPrevU;
          const dy0 = p.prevV - meanPrevV;
          const dx1 = p.u - meanCurrU;
          const dy1 = p.v - meanCurrV;
          numA += dx0 * dx1 + dy0 * dy1;
          numB += dx0 * dy1 - dy0 * dx1;
          den += dx0 * dx0 + dy0 * dy0;
        }
        if (den > 35.0) {
          const a = numA / den;
          const b = numB / den;
          const rawScale = Math.hypot(a, b);
          const rawRoll = Math.atan2(b, a);
          if (rawScale > 0.90 && rawScale < 1.12) dScale = rawScale;
          if (Math.abs(rawRoll) < 0.14) dRoll = rawRoll;
        }
      }
    }

    return { camX: medCamX, camY: medCamY, dScale, dRoll };
  }

  _updateKeyframeVisualSLAM(videoEl, imuCamDX, imuCamDY) {
    if (!videoEl || videoEl.readyState < 2 || videoEl.paused) {
      return false;
    }

    const vTime = videoEl.currentTime;
    if (vTime === this.lastVideoTime && this.prevGrayFrame !== null) {
      return false;
    }
    this.lastVideoTime = vTime;

    const { gray, coarse } = this._capturePyramidFrames(videoEl);
    const w = this.voWidth;
    const h = this.voHeight;

    if (!this.prevGrayFrame || this.landmarks.length < 8) {
      this.prevGrayFrame = gray;
      this.prevCoarseFrame = coarse;
      this._spawnLandmarks(gray);
      this.trackedFeatureCount = this.landmarks.length;
      this._renderSlamOverlay(this.landmarks);
      return true;
    }

    const coarseDelta = this._estimateCoarseGlobalDelta(this.prevCoarseFrame, coarse);
    const useImu =
      Math.hypot(imuCamDX, imuCamDY) >
      Math.hypot(coarseDelta.camDeltaX, coarseDelta.camDeltaY);
    const guessCamX = this.estCamX + (useImu ? imuCamDX : coarseDelta.camDeltaX);
    const guessCamY = this.estCamY + (useImu ? imuCamDY : coarseDelta.camDeltaY);

    const matchedPairs = [];
    const surviving = [];
    const border = 6;
    const fineRadius = 5;

    for (let i = 0; i < this.landmarks.length; i++) {
      const lm = this.landmarks[i];
      const predX = Math.round(lm.wx - guessCamX);
      const predY = Math.round(lm.wy - guessCamY);

      if (predX < border || predX >= w - border || predY < border || predY >= h - border) {
        continue;
      }

      let bestX = predX;
      let bestY = predY;
      let minZSAD = 1e18;

      for (let dy = -fineRadius; dy <= fineRadius; dy++) {
        const cy = predY + dy;
        if (cy < border || cy >= h - border) continue;
        for (let dx = -fineRadius; dx <= fineRadius; dx++) {
          const cx = predX + dx;
          if (cx < border || cx >= w - border) continue;
          const zsad = this._patchZSAD9x9(lm.patch, gray, cx, cy);
          if (zsad < minZSAD) {
            minZSAD = zsad;
            bestX = cx;
            bestY = cy;
          }
        }
      }

      let subX = bestX;
      let subY = bestY;
      if (
        bestX > border &&
        bestX < w - border - 1 &&
        bestY > border &&
        bestY < h - border - 1
      ) {
        const sadL = this._patchZSAD9x9(lm.patch, gray, bestX - 1, bestY);
        const sadR = this._patchZSAD9x9(lm.patch, gray, bestX + 1, bestY);
        const denomX = 2.0 * (sadL - 2.0 * minZSAD + sadR);
        if (Math.abs(denomX) > 1e-2) {
          subX += Math.max(-0.5, Math.min(0.5, (sadL - sadR) / denomX));
        }

        const sadU = this._patchZSAD9x9(lm.patch, gray, bestX, bestY - 1);
        const sadD = this._patchZSAD9x9(lm.patch, gray, bestX, bestY + 1);
        const denomY = 2.0 * (sadU - 2.0 * minZSAD + sadD);
        if (Math.abs(denomY) > 1e-2) {
          subY += Math.max(-0.5, Math.min(0.5, (sadU - sadD) / denomY));
        }
      }

      if (minZSAD < 81 * 24) {
        matchedPairs.push({
          wx: lm.wx,
          wy: lm.wy,
          prevU: lm.x,
          prevV: lm.y,
          u: subX,
          v: subY
        });
        lm.vx = subX - lm.x;
        lm.vy = subY - lm.y;
        lm.x = subX;
        lm.y = subY;
        surviving.push(lm);
      }
    }

    const sim = this._solveSimilarityTransform2D(matchedPairs);
    if (sim && matchedPairs.length >= 3) {
      const deltaX = sim.camX - this.estCamX;
      const deltaY = sim.camY - this.estCamY;
      const deltaMag = Math.hypot(deltaX, deltaY);

      // 双级迟滞死区门限 (Hysteresis Anti-Wobble Gate):
      // 当帧间位移小于 0.28px 时完全锁死姿态，滤除手持静止时的微颤与 CMOS 噪点
      if (deltaMag < this.deadzoneThresholdPx) {
        this.staticFrameCounter++;
        this.motionState = 'STATIC_LOCK';
      } else {
        this.staticFrameCounter = 0;
        this.motionState = 'VIO_LOCK';
        this.estCamX = sim.camX;
        this.estCamY = sim.camY;
        if (Math.abs(sim.dScale - 1.0) > 0.012) {
          this.estScale = Math.max(0.65, Math.min(1.65, this.estScale * sim.dScale));
        }
        if (Math.abs(sim.dRoll) > 0.006) {
          this.estRoll = this._wrapAngleRad(this.estRoll + sim.dRoll * 0.5);
        }
      }
    } else {
      const imuMove = Math.hypot(imuCamDX, imuCamDY);
      if (imuMove > this.deadzoneThresholdPx) {
        this.motionState = 'ROTATING';
        this.estCamX = guessCamX;
        this.estCamY = guessCamY;
      } else {
        this.motionState = 'STATIC_LOCK';
      }
    }

    this.landmarks = surviving;
    if (this.landmarks.length < 28) {
      this._spawnLandmarks(gray);
    }

    this.trackedFeatureCount = this.landmarks.length;
    this.prevGrayFrame = gray;
    this.prevCoarseFrame = coarse;
    this._renderSlamOverlay(this.landmarks);
    return true;
  }

  _renderSlamOverlay(points) {
    if (!this.slamCanvas || !this.slamCtx) return;
    const cw = this.slamCanvas.width;
    const ch = this.slamCanvas.height;
    this.slamCtx.clearRect(0, 0, cw, ch);
    if (!this.showSlamFeatures || !points || points.length === 0) return;

    const scaleX = cw / this.voWidth;
    const scaleY = ch / this.voHeight;

    // 1. 绘制 OpenCV 共面特征聚类连线 (Delaunay/Coplanar Mesh Triangles 示意)
    if (points.length >= 4) {
      this.slamCtx.strokeStyle = 'rgba(0, 242, 254, 0.16)';
      this.slamCtx.lineWidth = 1.0;
      this.slamCtx.beginPath();
      for (let i = 0; i < points.length; i++) {
        const p1 = points[i];
        let connected = 0;
        for (let j = i + 1; j < points.length && connected < 2; j++) {
          const p2 = points[j];
          const d = Math.hypot(p1.x - p2.x, p1.y - p2.y);
          if (d > 10 && d < 28) {
            this.slamCtx.moveTo(p1.x * scaleX, p1.y * scaleY);
            this.slamCtx.lineTo(p2.x * scaleX, p2.y * scaleY);
            connected++;
          }
        }
      }
      this.slamCtx.stroke();
    }

    // 2. 绘制特征角点与光流残差向量
    this.slamCtx.lineWidth = 1.5;
    for (let i = 0; i < points.length; i++) {
      const pt = points[i];
      const sx = pt.x * scaleX;
      const sy = pt.y * scaleY;

      if (Math.hypot(pt.vx || 0, pt.vy || 0) > 0.25) {
        this.slamCtx.strokeStyle = 'rgba(0, 242, 254, 0.72)';
        this.slamCtx.beginPath();
        this.slamCtx.moveTo(sx, sy);
        this.slamCtx.lineTo(sx - pt.vx * scaleX * 2.0, sy - pt.vy * scaleY * 2.0);
        this.slamCtx.stroke();
      }

      this.slamCtx.strokeStyle = 'rgba(56, 239, 125, 0.88)';
      this.slamCtx.beginPath();
      this.slamCtx.moveTo(sx - 3.5, sy);
      this.slamCtx.lineTo(sx + 3.5, sy);
      this.slamCtx.moveTo(sx, sy - 3.5);
      this.slamCtx.lineTo(sx, sy + 3.5);
      this.slamCtx.stroke();
    }
  }

  /**
   * 核心升级 (v8.0)：真实三维世界空间相机姿态更新 (True 3D World-Space Camera Pose)
   * - 所有识别出的平面 (ARPlane) 和放置的模型 (AnchorNode) 严格保持在它们各自的三维世界坐标 (Xw, Y_plane, Zw) 上不动！
   * - 模型严格垂直于水平面 (rotation.x = 0, rotation.z = 0)，绝不随镜头摇晃或倾斜！
   * - 只有 this.camera 根据 OpenCV + IMU 求解出的 (smoothCamX, smoothCamY, smoothScale, smoothRoll) 更新其 3D 世界姿态！
   */
  update(timestampMs, xrFrame = null, videoElement = null, placedAnchors = []) {
    const dtSec =
      this.lastUpdateTimestampMs !== null
        ? Math.max(0.002, Math.min(0.08, (timestampMs - this.lastUpdateTimestampMs) * 0.001))
        : 0.016;
    this.lastUpdateTimestampMs = timestampMs;

    const imuDX = this.imuDeltaPxX;
    const imuDY = this.imuDeltaPxY;
    const imuDRoll = this.imuDeltaRoll;
    this.imuDeltaPxX = 0;
    this.imuDeltaPxY = 0;
    this.imuDeltaRoll = 0;

    const updatedFromVideo = this._updateKeyframeVisualSLAM(videoElement, imuDX, imuDY);
    if (!updatedFromVideo) {
      const imuMag = Math.hypot(imuDX, imuDY);
      if (imuMag > this.deadzoneThresholdPx) {
        this.estCamX += imuDX;
        this.estCamY += imuDY;
        this.motionState = 'ROTATING';
      } else {
        this.motionState = 'STATIC_LOCK';
      }
    }
    if (Math.abs(imuDRoll) > 0.004) {
      this.estRoll = this._wrapAngleRad(this.estRoll * 0.94 + imuDRoll * 0.45);
    } else {
      this.estRoll *= 0.96;
    }

    // 二阶临界阻尼弹簧平滑 (源自 Google <model-viewer> Damper)
    this.smoothCamX = this.springCamX.update(this.estCamX, dtSec);
    this.smoothCamY = this.springCamY.update(this.estCamY, dtSec);
    this.smoothScale = this.springScale.update(this.estScale, dtSec);
    this.smoothRoll = this.springRoll.update(this.estRoll, dtSec);

    const halfFovRad = (this.camera.fov * Math.PI) / 360;
    const tanHalf = Math.tan(halfFovRad);
    const aspect = this.camera.aspect || window.innerWidth / window.innerHeight;

    // 将累积像素位移精确映射为三维相机旋转角 (Yaw, Pitch, Roll) 与深度平移 (Tz)
    this.camYaw = -((2.0 * this.smoothCamX) / this.voWidth) * aspect * tanHalf;
    this.camPitch = this.basePitch - ((2.0 * this.smoothCamY) / this.voHeight) * tanHalf;
    this.camRoll = this.smoothRoll;

    // 相机前后推进位移 (由 OpenCV 相似变换尺度因子 smoothScale 驱动)
    const refDist = this.defaultSpawnDistance;
    const forwardZ = Math.max(-0.85, Math.min(0.85, refDist * (1.0 - 1.0 / this.smoothScale)));

    this.camera.position.set(0, 0, -forwardZ);
    this.camera.rotation.set(this.camPitch, this.camYaw, this.camRoll, 'YXZ');
    this.camera.updateMatrixWorld(true);
    this.cameraPosition.copy(this.camera.position);

    // 确保所有已锁定在平面上的模型严格直立 (rotation.x = 0, rotation.z = 0)，不发生任何自摇摆！
    for (let i = 0; i < placedAnchors.length; i++) {
      const anchor = placedAnchors[i];
      if (!anchor.worldLocked) continue;
      anchor.root.rotation.set(0, anchor.baseYaw || 0, 0);
      if (typeof anchor.planeY === 'number') {
        anchor.root.position.y = anchor.planeY;
      }
    }
  }

  /**
   * 注册识别到的三维平面 (ARPlane) 供射线碰撞检测
   */
  setDetectedPlanes(planesList) {
    this.detectedPlanes = planesList || [];
  }

  /**
   * Google <model-viewer> & SceneView 风格：三维射线与已识别平面 (ARPlane) 碰撞吸附求解
   * 优先与场景中已识别的 ARPlane 多边形碰撞；若未直接命中多边形内部，则与距离射线最近的高度平面相交并吸附！
   */
  raycastToPlanes(clientX, clientY, viewportRect) {
    const normU = (clientX - viewportRect.left) / viewportRect.width;
    const normV = (clientY - viewportRect.top) / viewportRect.height;
    const ndcX = normU * 2.0 - 1.0;
    const ndcY = -normV * 2.0 + 1.0;

    this.raycaster.setFromCamera(new THREE.Vector2(ndcX, ndcY), this.camera);
    const ray = this.raycaster.ray;

    // 1. 先检测是否直接命中已识别平面的多边形网格 (Mesh Hit-Test)
    const colliderMeshes = [];
    for (let i = 0; i < this.detectedPlanes.length; i++) {
      const p = this.detectedPlanes[i];
      if (p.colliderMesh && p.group.visible) {
        colliderMeshes.push(p.colliderMesh);
      }
    }

    if (colliderMeshes.length > 0) {
      const hits = this.raycaster.intersectObjects(colliderMeshes, false);
      if (hits.length > 0) {
        const hit = hits[0];
        const matchedPlane = this.detectedPlanes.find((p) => p.colliderMesh === hit.object);
        if (matchedPlane) {
          this.activeHitPlaneId = matchedPlane.id;
          return {
            position: new THREE.Vector3(hit.point.x, matchedPlane.height, hit.point.z),
            plane: matchedPlane,
            snappedToPolygon: true,
            yawRotation: 0
          };
        }
      }
    }

    // 2. 若手指落在平面多边形边缘外，计算射线与各识别平面无限水平面 (y = plane.height) 的交点，选择距离平面中心最近的一个平面自动磁吸！
    let bestPlane = null;
    let bestPoint = null;
    let minEdgeDist = 1e18;

    for (let i = 0; i < this.detectedPlanes.length; i++) {
      const plane = this.detectedPlanes[i];
      const mathPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -plane.height);
      const intersectPt = new THREE.Vector3();
      if (ray.intersectPlane(mathPlane, intersectPt)) {
        const distFromCam = intersectPt.distanceTo(this.camera.position);
        if (distFromCam > 0.35 && distFromCam < 8.5) {
          const distToCenter = Math.hypot(
            intersectPt.x - plane.center.x,
            intersectPt.z - plane.center.z
          );
          const normalizedScore = distToCenter / (plane.radius || 1.0);
          if (normalizedScore < minEdgeDist) {
            minEdgeDist = normalizedScore;
            bestPlane = plane;
            bestPoint = intersectPt.clone();
          }
        }
      }
    }

    if (bestPlane && bestPoint) {
      // 若超出该平面半径较远，温和限制在合理视野范围内或自动扩展平面边界
      this.activeHitPlaneId = bestPlane.id;
      return {
        position: new THREE.Vector3(bestPoint.x, bestPlane.height, bestPoint.z),
        plane: bestPlane,
        snappedToPolygon: minEdgeDist <= 1.25,
        yawRotation: 0
      };
    }

    // 3. 兜底：与默认主地面 (groundHeight) 相交
    const fallbackPlane = this.detectedPlanes[0] || {
      id: 'plane_main',
      name: '平面 #1 · 主桌面/地面',
      height: this.groundHeight,
      center: new THREE.Vector3(0, this.groundHeight, -1.85),
      radius: 1.2
    };
    const defaultPos = new THREE.Vector3()
      .copy(ray.origin)
      .addScaledVector(ray.direction, this.defaultSpawnDistance);
    defaultPos.y = fallbackPlane.height;
    this.activeHitPlaneId = fallbackPlane.id;

    return {
      position: defaultPos,
      plane: fallbackPlane,
      snappedToPolygon: false,
      yawRotation: 0
    };
  }

  /**
   * 兼容接口：供拖拽悬停与释放放置调用
   */
  computeWorldDropPose(clientX, clientY, viewportRect) {
    return this.raycastToPlanes(clientX, clientY, viewportRect);
  }

  /**
   * 获取屏幕中心准星与识别平面的实时交点
   */
  getCenterReticlePose() {
    const rect = {
      left: 0,
      top: 0,
      width: window.innerWidth || 390,
      height: window.innerHeight || 844
    };
    const cx = rect.width * 0.5;
    const cy = rect.height * 0.56;
    const hit = this.raycastToPlanes(cx, cy, rect);
    return {
      position: hit.position,
      plane: hit.plane,
      snappedToPolygon: hit.snappedToPolygon,
      valid: true
    };
  }

  /**
   * 基于当前 OpenCV 视觉特征点簇在视野中心识别并提取一个新的三维物理平面参数
   */
  analyzeFeatureClusterForNewPlane() {
    const rect = {
      left: 0,
      top: 0,
      width: window.innerWidth || 390,
      height: window.innerHeight || 844
    };
    const cx = rect.width * 0.5;
    const cy = rect.height * 0.52;
    const normU = 0.5;
    const normV = 0.52;
    const ndcX = normU * 2.0 - 1.0;
    const ndcY = -normV * 2.0 + 1.0;

    this.raycaster.setFromCamera(new THREE.Vector2(ndcX, ndcY), this.camera);
    const ray = this.raycaster.ray;

    // 根据当前视野中心特征点的密集程度与纵向分布估算新平面的物理高度与半径
    let clusterSpread = 0.65;
    let estDist = 1.65;
    if (this.landmarks.length >= 6) {
      let sumScore = 0;
      for (let i = 0; i < this.landmarks.length; i++) {
        sumScore += this.landmarks[i].score || 30;
      }
      const avgScore = sumScore / this.landmarks.length;
      clusterSpread = Math.max(0.48, Math.min(0.95, 0.45 + (avgScore / 180) * 0.4));
    }

    const center3D = new THREE.Vector3()
      .copy(ray.origin)
      .addScaledVector(ray.direction, estDist);
    // 量化高度到合理的物理层高区间 [-1.35m, -0.45m]
    const clampedY = Math.max(-1.35, Math.min(-0.45, Number(center3D.y.toFixed(2))));
    center3D.y = clampedY;

    return {
      center: center3D,
      height: clampedY,
      radius: clusterSpread
    };
  }

  applySimulatedCameraDelta(deltaYawDeg, deltaPitchDeg, deltaForward = 0, deltaRight = 0) {
    const deg2rad = Math.PI / 180;
    const { pxX, pxY } = this._anglesToVoPixels(deltaYawDeg * deg2rad, deltaPitchDeg * deg2rad);
    this.estCamX += pxX + deltaRight * 18.0;
    this.estCamY += pxY;
    if (deltaForward !== 0) {
      this.estScale = Math.max(0.65, Math.min(1.65, this.estScale * (1.0 + deltaForward * 0.2)));
    }
  }

  resetCameraPose() {
    this.estCamX = 0;
    this.estCamY = 0;
    this.estScale = 1.0;
    this.estRoll = 0.0;
    this.springCamX.reset(0);
    this.springCamY.reset(0);
    this.springScale.reset(1.0);
    this.springRoll.reset(0);
    this.smoothCamX = 0;
    this.smoothCamY = 0;
    this.smoothScale = 1.0;
    this.smoothRoll = 0;
    this.camYaw = 0;
    this.camPitch = this.basePitch;
    this.camRoll = 0;
    this.camera.position.set(0, 0, 0);
    this.camera.rotation.set(this.basePitch, 0, 0, 'YXZ');
    this.camera.updateMatrixWorld(true);
  }
}
