# ARviewer (HoloCanvas AR) — 全息空间建模相机

<div align="center">
  <p>
    <img src="https://img.shields.io/badge/Status-Prototype_(半成品_/_有待完善)-yellow.svg" alt="Status" />
    <img src="https://img.shields.io/badge/Tech-WebXR_+_Three.js-blue.svg" alt="Tech" />
    <img src="https://img.shields.io/badge/Framework-Capacitor_Mobile-orange.svg" alt="Framework" />
    <img src="https://img.shields.io/badge/Tracking-6DoF_SLAM_+_VIO-brightgreen.svg" alt="Tracking" />
  </p>
</div>

> [!NOTE]
> **项目状态声明 (Work in Progress)**：
> 本项目为面向移动端的实验性 AR 全息相机原型工程（**半成品，功能与适配有待进一步完善**）。已实现基础的 WebXR 硬件级平面检测、单目 VIO/光流降级追踪算法、1€ Filter 防抖滤波以及本地 3D 模型拖拽空间锚定管线，供技术研究与后续迭代参考。

---

## 核心设计与定位能力

**ARviewer** 旨在实现类似 Pokémon GO 体验的沉浸式空间建模相机：支持从本地导入各类 3D 模型（`.glb` / `.gltf` / `.obj` / `.stl` / `.fbx`），直接从底部资产抽屉拖拽至物理世界的摄像头画面中完成空间锚定（World Anchoring）。

### 三级自适应空间追踪管线 (`SpatialTracker.js`)

为了在不同硬件条件下保持模型在物理空间中相对静止，系统设计了自适应回退追踪：

1. **第一级：硬件级 6DoF SLAM 锚定（原生 WebXR / ARCore 通道）**
   - 依赖设备支持 `immersive-ar` 会话，通过 `XRHitTestSource` 实时探测物理平面交点并创建持久世界锚点 `XRAnchor`。
2. **第二级：IMU 互补滤波 + 单目光流视觉里程计（Sensor Fusion VIO 降级通道）**
   - 在不支持原生 WebXR 的移动浏览器或普通 WebView 中降级运行：
     - **3DoF 姿态解算**：融合设备陀螺仪与加速度计，通过四元数变换校准屏幕方向。
     - **平移推算**：采用金字塔 Lucas-Kanade 局部光流法提取特征位移，剔除纯旋转视差，通过中值鲁棒估计估算平面位移。
3. **第三级：射线-地平面求交 + 1€ Filter（One-Euro Filter）自适应低通防抖**
   - 拖拽松手瞬间发射空间射线，与虚拟物理地基面求交；通过动态截止频率低通滤波，确保静止时模型稳定不飘移、快速移动时低延迟跟随。

---

## 工程目录结构

```text
ARviewer/
├── index.html                        # 移动端全屏交互与 Canvas 毛玻璃 UI 布局
├── package.json                      # 依赖配置 (Three.js, Vite, Capacitor)
├── capacitor.config.json             # Android / iOS 原生打包桥接配置
├── vite.config.js                    # 构建与本地 HTTPS 开发调试配置
├── public/
│   └── models/
│       └── HoloPokeball.obj          # 内置测试模型：全息捕捉球
├── src/
│   ├── main.js                       # 主控制器：调度 AR 相机、渲染循环与触摸事件
│   ├── algorithms/
│   │   ├── SpatialTracker.js         # 三级空间追踪核心：WebXR + IMU/光流融合 + 射线求交
│   │   └── OneEuroFilter.js          # 1€ Filter 自适应低通滤波 (位置与姿态四元数防抖)
│   ├── loaders/
│   │   └── ModelLoaderPipeline.js    # 多格式 3D 加载管线与包围盒自适应归一化
│   ├── shaders/
│   │   └── HologramMaterial.js       # 全息 Shader：Fresnel 边缘辉光 + 动态扫描线
│   ├── ui/
│   │   └── CanvasWorkspaceUI.js      # 资产抽屉拖拽、手势缩放旋转与快门控制
│   └── styles/
│       └── canvas-ui.css             # 响应式移动端暗色玻璃态样式表
├── android/                          # Capacitor 生成的 Android 原生工程包
└── native-android-sceneview/
    └── HologramArScreen.kt           # 原生 Jetpack Compose + Google SceneView (ARCore) 参考源码
```

---

## 快速上手与本地调试

### 1. Web 端本地调试
```bash
# 安装依赖
npm install

# 启动本地 HTTPS 服务（移动端传感器调用必须依赖 HTTPS 或 localhost）
npm run dev -- --host
```

### 2. 打包为 Android 原生应用
```bash
# 编译前端构建产物
npm run build

# 同步至 Android Capacitor 工程
npx cap sync android

# 在 Android Studio 中打开并构建 APK
npx cap open android
```

---

## 当前已知局限与待完善项 (TODOs)

- [ ] **平面网格重建**：目前普通浏览器降级模式仅依赖经验地平面方程，弱纹理场景下的单目光流位移累计误差较大，需引入特征点回环检测。
- [ ] **多模型同屏性能优化**：当前针对复杂网格模型的 Draw Call 与阴影贴图尚未进行深度合批。
- [ ] **手势冲突处理**：移动端部分浏览器中单指拖动模型容易触发系统级页面下拉刷新，手势拦截需进一步增强。
- [ ] **Native 原生层对接**：`native-android-sceneview/` 目前仅为代码对照参考，尚未与 Webview 实现无缝双向通信。
