/**
 * OneEuroFilter.js
 * 1€ 自适应低通滤波器 (Casiez et al., CHI 2012)
 * 用于消除手机陀螺仪、加速度计与单目光流估计中的高频抖动，同时在快速移动相机时保持极低延迟。
 */
import * as THREE from 'three';

class LowPassFilter {
  constructor(alpha, initVal = 0) {
    this.y = initVal;
    this.s = initVal;
    this.initialized = false;
    this.setAlpha(alpha);
  }

  setAlpha(alpha) {
    this.alpha = Math.min(1.0, Math.max(0.0001, alpha));
  }

  filter(value) {
    if (!this.initialized) {
      this.s = value;
      this.y = value;
      this.initialized = true;
      return value;
    }
    this.s = this.alpha * value + (1.0 - this.alpha) * this.s;
    this.y = value;
    return this.s;
  }

  filterWithAlpha(value, alpha) {
    this.setAlpha(alpha);
    return this.filter(value);
  }

  lastValue() {
    return this.y;
  }
}

export class OneEuroFilterScalar {
  /**
   * @param {number} freq 采样频率 (Hz)
   * @param {number} minCutoff 最小截止频率 (Hz)，越小静止时越稳定
   * @param {number} beta 速度系数，越大快速运动时延迟越低
   * @param {number} dCutoff 导数截止频率 (Hz)
   */
  constructor(freq = 60, minCutoff = 1.0, beta = 0.015, dCutoff = 1.0) {
    this.freq = freq;
    this.minCutoff = minCutoff;
    this.beta = beta;
    this.dCutoff = dCutoff;
    this.x = new LowPassFilter(this._alpha(minCutoff));
    this.dx = new LowPassFilter(this._alpha(dCutoff));
    this.lastTime = null;
  }

  _alpha(cutoff) {
    const te = 1.0 / this.freq;
    const tau = 1.0 / (2 * Math.PI * cutoff);
    return 1.0 / (1.0 + tau / te);
  }

  filter(value, timestampSec = null) {
    if (this.lastTime !== null && timestampSec !== null) {
      const dt = timestampSec - this.lastTime;
      if (dt > 1e-5 && dt < 1.0) {
        this.freq = 1.0 / dt;
      }
    }
    this.lastTime = timestampSec;

    const dValue = this.x.initialized ? (value - this.x.lastValue()) * this.freq : 0.0;
    const edValue = this.dx.filterWithAlpha(dValue, this._alpha(this.dCutoff));
    const cutoff = this.minCutoff + this.beta * Math.abs(edValue);
    return this.x.filterWithAlpha(value, this._alpha(cutoff));
  }
}

export class OneEuroFilterVector3 {
  constructor(freq = 60, minCutoff = 1.2, beta = 0.02, dCutoff = 1.0) {
    this.fx = new OneEuroFilterScalar(freq, minCutoff, beta, dCutoff);
    this.fy = new OneEuroFilterScalar(freq, minCutoff, beta, dCutoff);
    this.fz = new OneEuroFilterScalar(freq, minCutoff, beta, dCutoff);
  }

  filter(vec3, timestampSec = null) {
    return new THREE.Vector3(
      this.fx.filter(vec3.x, timestampSec),
      this.fy.filter(vec3.y, timestampSec),
      this.fz.filter(vec3.z, timestampSec)
    );
  }
}

export class OneEuroFilterQuaternion {
  constructor(freq = 60, minCutoff = 1.5, beta = 0.04) {
    this.freq = freq;
    this.minCutoff = minCutoff;
    this.beta = beta;
    this.currentQuat = null;
    this.lastTime = null;
  }

  filter(targetQuat, timestampSec = null) {
    if (!this.currentQuat) {
      this.currentQuat = targetQuat.clone();
      this.lastTime = timestampSec;
      return this.currentQuat.clone();
    }

    let dt = 1.0 / this.freq;
    if (timestampSec !== null && this.lastTime !== null) {
      dt = Math.max(0.001, Math.min(0.2, timestampSec - this.lastTime));
    }
    this.lastTime = timestampSec;

    // 确保四元数处于同一半球，避免双倍覆盖翻转
    if (this.currentQuat.dot(targetQuat) < 0) {
      targetQuat = new THREE.Quaternion(
        -targetQuat.x,
        -targetQuat.y,
        -targetQuat.z,
        -targetQuat.w
      );
    }

    const angleDiff = 2.0 * Math.acos(Math.min(1.0, Math.abs(this.currentQuat.dot(targetQuat))));
    const angularSpeed = angleDiff / dt;
    const cutoff = this.minCutoff + this.beta * angularSpeed;
    const tau = 1.0 / (2.0 * Math.PI * cutoff);
    const alpha = Math.min(1.0, Math.max(0.02, 1.0 / (1.0 + tau / dt)));

    this.currentQuat.slerp(targetQuat, alpha);
    this.currentQuat.normalize();
    return this.currentQuat.clone();
  }
}
