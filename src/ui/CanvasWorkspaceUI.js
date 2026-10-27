/**
 * CanvasWorkspaceUI.js (v8.0 Ultra-Minimalist "Zero-Chrome" AR Camera HUD + Multi-Plane Controls)
 * - 顶部：左侧显示已识别平面数与防抖锁定状态，右侧配备「识别新平面 📐」「显隐平面网格 🕸️」「材质切换 ✨」「隐藏底栏 👁」「清空 🗑」纯图标圆钮
 * - 底部：58px 悬浮晶球底座，按住圆标拖入画面即自动射线吸附到目标三维平面 (ARPlane)
 * - 选中模型时：浮现 38px 单行图标胶囊（🔒锁定 / 📐切换平面 / − / ＋ / ↻旋转 / 调色圆点 / ✕移除）
 */
export class CanvasWorkspaceUI {
  constructor(callbacks) {
    this.cb = callbacks;
    this.assets = [];
    this.draggingAssetId = null;
    this.shaderModes = ['hologram', 'pbr', 'wireframe'];
    this.shaderIdx = 0;
    this.holoColors = ['#00f2fe', '#38ef7d', '#ffb703', '#ff3366', '#9d4edd'];
    this.colorIdx = 0;

    this.ghostEl = document.getElementById('drag-ghost');
    this.assetListEl = document.getElementById('asset-cards-track');
    this.inspectorPanelEl = document.getElementById('canvas-inspector-pill');
    this.dockEl = document.getElementById('bottom-canvas-dock');
    this.toastBannerEl = document.getElementById('ar-toast-banner');

    this._initFileUploadBindings();
    this._initToolbarAndPanels();
    this._initInspectorControls();
    this._initViewportTouchGestures();
    this._initSnapshotModal();
  }

  showToast(message) {
    if (!this.toastBannerEl) return;
    this.toastBannerEl.textContent = message;
    this.toastBannerEl.classList.remove('hidden');
    clearTimeout(this._toastTimer);
    this._toastTimer = setTimeout(() => {
      this.toastBannerEl.classList.add('hidden');
    }, 1800);
  }

  registerAssetCard(assetItem) {
    this.assets.push(assetItem);
    this._renderAssetCards();
  }

  _renderAssetCards() {
    this.assetListEl.innerHTML = '';
    this.assets.forEach((assetItem) => {
      const orb = document.createElement('div');
      orb.className = 'model-orb';
      orb.dataset.assetId = assetItem.id;
      orb.title = `按住拖入画面平面: ${assetItem.name}`;
      const shortName = assetItem.name.replace(/\.[a-z0-9]+$/i, '').slice(0, 4);
      orb.innerHTML = `
        <span class="orb-glyph">${assetItem.icon || '◈'}</span>
        <span class="orb-short-label">${shortName}</span>
      `;
      this._bindOrbDragEvents(orb, assetItem);
      this.assetListEl.appendChild(orb);
    });
  }

  _bindOrbDragEvents(orbEl, assetItem) {
    let activePointerId = null;

    orbEl.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      activePointerId = e.pointerId;
      orbEl.setPointerCapture(e.pointerId);
      this.draggingAssetId = assetItem.id;

      this.ghostEl.classList.remove('hidden');
      this._moveGhost(e.clientX, e.clientY);
      this.cb.onDragMoveOverViewport(e.clientX, e.clientY, true);
    });

    orbEl.addEventListener('pointermove', (e) => {
      if (e.pointerId !== activePointerId || !this.draggingAssetId) return;
      this._moveGhost(e.clientX, e.clientY);
      this.cb.onDragMoveOverViewport(e.clientX, e.clientY, true);
    });

    const finishDrag = (e) => {
      if (e.pointerId !== activePointerId || !this.draggingAssetId) return;
      orbEl.releasePointerCapture(e.pointerId);
      this.ghostEl.classList.add('hidden');

      const dockRect = this.dockEl.getBoundingClientRect();
      if (e.clientY < dockRect.top + 8) {
        this.cb.onDropAssetToViewport(this.draggingAssetId, e.clientX, e.clientY);
      } else {
        this.cb.onDragMoveOverViewport(0, 0, false);
      }

      this.draggingAssetId = null;
      activePointerId = null;
    };

    orbEl.addEventListener('pointerup', finishDrag);
    orbEl.addEventListener('pointercancel', finishDrag);
  }

  _moveGhost(x, y) {
    this.ghostEl.style.transform = `translate3d(${x - 28}px, ${y - 28}px, 0)`;
  }

  _initFileUploadBindings() {
    const fileInput = document.getElementById('local-model-input');
    const importBtn = document.getElementById('btn-import-model');

    importBtn.addEventListener('click', () => fileInput.click());
    fileInput.addEventListener('change', (e) => {
      if (e.target.files && e.target.files.length > 0) {
        this.cb.onImportFiles(e.target.files);
        fileInput.value = '';
      }
    });

    window.addEventListener('dragover', (e) => {
      e.preventDefault();
      this.cb.onDragMoveOverViewport(e.clientX, e.clientY, true);
    });

    window.addEventListener('drop', async (e) => {
      e.preventDefault();
      if (e.dataTransfer && e.dataTransfer.files && e.dataTransfer.files.length > 0) {
        await this.cb.onImportFiles(e.dataTransfer.files, {
          dropNow: true,
          clientX: e.clientX,
          clientY: e.clientY
        });
      }
    });
  }

  _initToolbarAndPanels() {
    const scanPlaneBtn = document.getElementById('btn-scan-plane');
    if (scanPlaneBtn) {
      scanPlaneBtn.addEventListener('click', () => {
        if (this.cb.onScanNewPlane) this.cb.onScanNewPlane();
      });
    }

    const togglePlanesBtn = document.getElementById('btn-toggle-planes');
    if (togglePlanesBtn) {
      togglePlanesBtn.addEventListener('click', () => {
        if (this.cb.onTogglePlanesVisibility) {
          const visible = this.cb.onTogglePlanesVisibility();
          togglePlanesBtn.classList.toggle('active', visible);
        }
      });
    }

    const cycleBtn = document.getElementById('btn-cycle-shader');
    cycleBtn.addEventListener('click', () => {
      this.shaderIdx = (this.shaderIdx + 1) % this.shaderModes.length;
      const mode = this.shaderModes[this.shaderIdx];
      const icons = { hologram: '✨', pbr: '☀️', wireframe: '🕸️' };
      cycleBtn.textContent = icons[mode];
      this.cb.onChangeRenderStyle(mode);
    });

    const slamPtsBtn = document.getElementById('btn-toggle-slam-pts');
    slamPtsBtn.addEventListener('click', () => {
      const enabled = this.cb.onToggleSlamPoints();
      slamPtsBtn.classList.toggle('pts-hidden', !enabled);
    });

    const toggleDockBtn = document.getElementById('btn-toggle-dock');
    toggleDockBtn.addEventListener('click', () => {
      const minimized = this.dockEl.classList.toggle('minimized');
      toggleDockBtn.classList.toggle('active', minimized);
    });

    document.getElementById('btn-clear-all').addEventListener('click', () => {
      this.cb.onClearAllAnchors();
    });

    document.getElementById('btn-shutter').addEventListener('click', () => {
      this.cb.onCaptureSnapshot();
    });
  }

  _initInspectorControls() {
    document.querySelectorAll('[data-gizmo-action]').forEach((btn) => {
      btn.addEventListener('click', () => {
        const action = btn.dataset.gizmoAction;
        if (action === 'delete') {
          this.cb.onDeleteSelected();
        } else {
          this.cb.onTransformSelected(action);
        }
      });
    });

    const colorDot = document.getElementById('btn-cycle-color');
    colorDot.addEventListener('click', () => {
      this.colorIdx = (this.colorIdx + 1) % this.holoColors.length;
      const hex = this.holoColors[this.colorIdx];
      colorDot.style.background = hex;
      this.cb.onChangeSelectedColor(hex);
    });
  }

  _initViewportTouchGestures() {
    const viewport = document.getElementById('ar-viewport-overlay');
    let initialPinchDist = null;
    let draggingPlacedModel = false;

    viewport.addEventListener('pointerdown', (e) => {
      if (e.target !== viewport) return;
      const hitAnchor = this.cb.onTapViewport(e.clientX, e.clientY);
      if (hitAnchor && !hitAnchor.worldLocked) {
        draggingPlacedModel = true;
        viewport.setPointerCapture(e.pointerId);
      }
    });

    viewport.addEventListener('pointermove', (e) => {
      if (draggingPlacedModel) {
        this.cb.onDragUnlockedModel(e.clientX, e.clientY);
      }
    });

    const endGesture = () => {
      draggingPlacedModel = false;
    };
    viewport.addEventListener('pointerup', endGesture);
    viewport.addEventListener('pointercancel', endGesture);

    viewport.addEventListener(
      'touchmove',
      (e) => {
        if (e.touches.length === 2) {
          e.preventDefault();
          const dx = e.touches[0].clientX - e.touches[1].clientX;
          const dy = e.touches[0].clientY - e.touches[1].clientY;
          const dist = Math.hypot(dx, dy);
          if (initialPinchDist !== null) {
            const ratio = dist / initialPinchDist;
            if (Math.abs(ratio - 1.0) > 0.015) {
              this.cb.onTransformSelected(ratio > 1 ? 'scale-up-fine' : 'scale-down-fine');
              initialPinchDist = dist;
            }
          } else {
            initialPinchDist = dist;
          }
        }
      },
      { passive: false }
    );

    viewport.addEventListener('touchend', () => {
      initialPinchDist = null;
    });
  }

  _initSnapshotModal() {
    const modal = document.getElementById('snapshot-modal');
    const closeBtn = document.getElementById('btn-close-snapshot');
    const downloadBtn = document.getElementById('btn-download-snapshot');

    closeBtn.addEventListener('click', () => modal.classList.add('hidden'));
    downloadBtn.addEventListener('click', () => {
      const img = document.getElementById('snapshot-preview-img');
      if (!img || !img.src) return;
      const link = document.createElement('a');
      link.download = `HoloCanvas_AR_${Date.now()}.png`;
      link.href = img.src;
      link.click();
    });
  }

  openSnapshotPreview(dataUrl) {
    const modal = document.getElementById('snapshot-modal');
    const img = document.getElementById('snapshot-preview-img');
    img.src = dataUrl;
    modal.classList.remove('hidden');
  }

  updateSelectionInspector(anchorEntry) {
    if (!anchorEntry) {
      this.inspectorPanelEl.classList.add('hidden');
      return;
    }
    this.inspectorPanelEl.classList.remove('hidden');

    const lockBtn = document.getElementById('btn-lock-toggle');
    if (lockBtn) {
      lockBtn.textContent = anchorEntry.worldLocked ? '🔒' : '🔓';
      lockBtn.classList.toggle('locked', !!anchorEntry.worldLocked);
    }

    const colorDot = document.getElementById('btn-cycle-color');
    if (colorDot && anchorEntry.holoColor) {
      colorDot.style.background = anchorEntry.holoColor;
    }
  }

  updateTelemetryHUD(telemetry) {
    const ptsEl = document.getElementById('hud-slam-points');
    if (!ptsEl) return;
    const planes = telemetry.planeCount || 3;
    const isSteady = telemetry.motionState === 'STATIC_LOCK';
    ptsEl.textContent = isSteady
      ? `${planes} 平面 · 稳态锁定`
      : `${planes} 平面 · 6DoF 追踪`;
  }
}
