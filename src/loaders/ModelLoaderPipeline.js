/**
 * ModelLoaderPipeline.js (v2.0)
 * 本地多格式 3D 建模文件解析 (.glb/.gltf/.obj/.stl/.fbx) 与内置全息预设生成器
 */
import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { DRACOLoader } from 'three/examples/jsm/loaders/DRACOLoader.js';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { FBXLoader } from 'three/examples/jsm/loaders/FBXLoader.js';

export class ModelLoaderPipeline {
  constructor() {
    this.gltfLoader = new GLTFLoader();
    const dracoLoader = new DRACOLoader();
    dracoLoader.setDecoderPath('https://www.gstatic.com/draco/versioned/decoders/1.5.7/');
    this.gltfLoader.setDRACOLoader(dracoLoader);

    this.objLoader = new OBJLoader();
    this.stlLoader = new STLLoader();
    this.fbxLoader = new FBXLoader();
  }

  async loadFromFile(file) {
    const ext = file.name.split('.').pop().toLowerCase();
    const objectUrl = URL.createObjectURL(file);
    try {
      return await this._loadByExtension(objectUrl, ext, file.name);
    } finally {
      URL.revokeObjectURL(objectUrl);
    }
  }

  async _loadByExtension(url, ext, name) {
    let rawRoot = null;
    let animations = [];

    if (ext === 'glb' || ext === 'gltf') {
      const gltf = await this.gltfLoader.loadAsync(url);
      rawRoot = gltf.scene || gltf.scenes[0];
      animations = gltf.animations || [];
    } else if (ext === 'obj') {
      rawRoot = await this.objLoader.loadAsync(url);
      this._assignDefaultPBRMaterial(rawRoot, 0x38ef7d);
    } else if (ext === 'stl') {
      const geometry = await this.stlLoader.loadAsync(url);
      geometry.computeVertexNormals();
      rawRoot = new THREE.Mesh(
        geometry,
        new THREE.MeshStandardMaterial({ color: 0x00d2ff, roughness: 0.28, metalness: 0.65 })
      );
    } else if (ext === 'fbx') {
      rawRoot = await this.fbxLoader.loadAsync(url);
      animations = rawRoot.animations || [];
    } else {
      throw new Error(`暂不支持 .${ext} 格式，请导入 .glb/.gltf/.obj/.stl/.fbx`);
    }

    return this._normalizeModelContainer(rawRoot, animations, name, ext);
  }

  _assignDefaultPBRMaterial(object3d, hexColor = 0x00f2fe) {
    object3d.traverse((child) => {
      if (child.isMesh) {
        child.material = new THREE.MeshStandardMaterial({
          color: hexColor,
          roughness: 0.3,
          metalness: 0.58
        });
      }
    });
  }

  _normalizeModelContainer(rawObject, animations, name, ext) {
    const box = new THREE.Box3().setFromObject(rawObject);
    const size = new THREE.Vector3();
    const center = new THREE.Vector3();
    box.getSize(size);
    box.getCenter(center);

    const maxDim = Math.max(size.x, size.y, size.z, 0.001);
    const targetPhysicalHeight = 0.72;
    const scaleFactor = targetPhysicalHeight / maxDim;

    rawObject.position.set(
      -center.x * scaleFactor,
      -box.min.y * scaleFactor,
      -center.z * scaleFactor
    );
    rawObject.scale.multiplyScalar(scaleFactor);

    let vertexCount = 0;
    rawObject.traverse((node) => {
      if (node.isMesh) {
        node.castShadow = true;
        node.receiveShadow = true;
        if (node.geometry && node.geometry.attributes.position) {
          vertexCount += node.geometry.attributes.position.count;
        }
      }
    });

    const anchorGroup = new THREE.Group();
    anchorGroup.name = name;

    // 底部全息能量基座光环
    const ringGeo = new THREE.RingGeometry(0.28, 0.34, 48);
    ringGeo.rotateX(-Math.PI / 2);
    const ringMat = new THREE.MeshBasicMaterial({
      color: 0x00f2fe,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.78
    });
    const baseRing = new THREE.Mesh(ringGeo, ringMat);
    baseRing.position.y = 0.005;
    baseRing.name = '__anchor_ring__';

    anchorGroup.add(baseRing, rawObject);

    return {
      root: anchorGroup,
      animations,
      meta: {
        name,
        format: ext.toUpperCase(),
        vertexCount,
        polyLabel: `${Math.max(1, Math.round(vertexCount / 100) / 10)}k Vtx`
      }
    };
  }

  createProceduralPreset(presetId) {
    const group = new THREE.Group();

    if (presetId === 'pokeball-holo') {
      const topMesh = new THREE.Mesh(
        new THREE.SphereGeometry(0.26, 36, 18, 0, Math.PI * 2, 0, Math.PI / 2),
        new THREE.MeshStandardMaterial({ color: 0xff3366, roughness: 0.18, metalness: 0.72 })
      );
      topMesh.position.y = 0.28;

      const botMesh = new THREE.Mesh(
        new THREE.SphereGeometry(0.26, 36, 18, 0, Math.PI * 2, Math.PI / 2, Math.PI / 2),
        new THREE.MeshStandardMaterial({ color: 0xf4f8ff, roughness: 0.25, metalness: 0.45 })
      );
      botMesh.position.y = 0.28;

      const coreGeo = new THREE.CylinderGeometry(0.07, 0.07, 0.06, 24);
      coreGeo.rotateX(Math.PI / 2);
      const coreMesh = new THREE.Mesh(
        coreGeo,
        new THREE.MeshStandardMaterial({
          color: 0x00f2fe,
          emissive: 0x00f2fe,
          emissiveIntensity: 0.9
        })
      );
      coreMesh.position.set(0, 0.28, 0.24);
      group.add(topMesh, botMesh, coreMesh);
      return this._normalizeModelContainer(group, [], '全息精灵胶囊.glb', 'glb');
    }

    if (presetId === 'pikachu-totem') {
      const body = new THREE.Mesh(
        new THREE.CapsuleGeometry(0.16, 0.22, 12, 24),
        new THREE.MeshStandardMaterial({ color: 0xffb703, roughness: 0.25, metalness: 0.45 })
      );
      body.position.y = 0.28;

      const earGeo = new THREE.ConeGeometry(0.05, 0.22, 16);
      const earMat = new THREE.MeshStandardMaterial({ color: 0xffb703, roughness: 0.25, metalness: 0.45 });
      const earL = new THREE.Mesh(earGeo, earMat);
      earL.position.set(-0.1, 0.52, 0);
      earL.rotation.z = 0.28;
      const earR = earL.clone();
      earR.position.set(0.1, 0.52, 0);
      earR.rotation.z = -0.28;

      group.add(body, earL, earR);
      return this._normalizeModelContainer(group, [], '雷电精灵图腾.glb', 'glb');
    }

    if (presetId === 'mecha-drone') {
      const body = new THREE.Mesh(
        new THREE.OctahedronGeometry(0.24, 1),
        new THREE.MeshStandardMaterial({
          color: 0x9d4edd,
          roughness: 0.18,
          metalness: 0.85,
          flatShading: true
        })
      );
      body.position.y = 0.35;

      const orbitGeo = new THREE.TorusGeometry(0.34, 0.022, 16, 48);
      orbitGeo.rotateX(Math.PI / 2.3);
      const orbit = new THREE.Mesh(
        orbitGeo,
        new THREE.MeshStandardMaterial({
          color: 0x00f2fe,
          emissive: 0x00a8ff,
          emissiveIntensity: 0.65
        })
      );
      orbit.position.y = 0.35;
      group.add(body, orbit);
      return this._normalizeModelContainer(group, [], '星际侦察机甲.gltf', 'gltf');
    }

    if (presetId === 'cyber-tesseract') {
      const outer = new THREE.Mesh(
        new THREE.TorusKnotGeometry(0.18, 0.045, 72, 16),
        new THREE.MeshStandardMaterial({ color: 0x38ef7d, roughness: 0.2, metalness: 0.8 })
      );
      outer.position.y = 0.34;
      group.add(outer);
      return this._normalizeModelContainer(group, [], '量子拓扑魔方.stl', 'stl');
    }

    const crystalGeo = new THREE.ConeGeometry(0.16, 0.62, 6);
    const crystalMat = new THREE.MeshStandardMaterial({
      color: 0x00f2fe,
      emissive: 0x005577,
      roughness: 0.15,
      metalness: 0.78,
      flatShading: true
    });
    const mainCrystal = new THREE.Mesh(crystalGeo, crystalMat);
    mainCrystal.position.y = 0.31;

    const sub1 = mainCrystal.clone();
    sub1.scale.set(0.6, 0.65, 0.6);
    sub1.position.set(0.14, 0.2, 0.08);
    sub1.rotation.z = -0.32;

    const sub2 = mainCrystal.clone();
    sub2.scale.set(0.5, 0.55, 0.5);
    sub2.position.set(-0.14, 0.17, -0.06);
    sub2.rotation.z = 0.36;

    group.add(mainCrystal, sub1, sub2);
    return this._normalizeModelContainer(group, [], '以太全息晶簇.obj', 'obj');
  }
}
