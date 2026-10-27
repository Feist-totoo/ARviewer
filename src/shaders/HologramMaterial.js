/**
 * HologramMaterial.js (v2.0)
 * 支持三种空间材质模式切换：
 * 1. 'hologram'  : 宝可梦 GO 风格菲涅尔边缘发光 + 动态全息扫描线 + 色彩主题定制
 * 2. 'pbr'       : 真实物理光影材质 + 地面实时阴影投射
 * 3. 'wireframe' : 激光雷达 (LiDAR) 空间点云网格扫描线框模式
 */
import * as THREE from 'three';

export function createHologramShaderMaterial(options = {}) {
  const primaryColor = new THREE.Color(options.primaryColor || 0x00f2fe);
  const rimColor = new THREE.Color(options.rimColor || 0x9d4edd);

  return new THREE.ShaderMaterial({
    uniforms: {
      uTime: { value: 0.0 },
      uPrimaryColor: { value: primaryColor },
      uRimColor: { value: rimColor },
      uScanlineDensity: { value: options.scanlineDensity || 44.0 },
      uOpacity: { value: options.opacity || 0.85 }
    },
    vertexShader: /* glsl */ `
      varying vec3 vWorldPosition;
      varying vec3 vNormal;
      uniform float uTime;

      void main() {
        vec4 worldPos = modelMatrix * vec4(position, 1.0);
        float glitch = sin(worldPos.y * 55.0 + uTime * 12.0) * 0.0022;
        float pulse = step(0.985, sin(uTime * 3.7 + worldPos.y * 4.0));
        worldPos.x += glitch * pulse * 5.0;

        vWorldPosition = worldPos.xyz;
        vNormal = normalize(normalMatrix * normal);
        gl_Position = projectionMatrix * viewMatrix * worldPos;
      }
    `,
    fragmentShader: /* glsl */ `
      uniform float uTime;
      uniform vec3 uPrimaryColor;
      uniform vec3 uRimColor;
      uniform float uScanlineDensity;
      uniform float uOpacity;

      varying vec3 vWorldPosition;
      varying vec3 vNormal;

      void main() {
        vec3 viewDir = normalize(cameraPosition - vWorldPosition);
        float fresnel = pow(1.0 - abs(dot(viewDir, normalize(vNormal))), 2.1);

        float scanlineFine = 0.5 + 0.5 * sin(vWorldPosition.y * uScanlineDensity - uTime * 5.8);
        float scanlineBroad = smoothstep(0.35, 0.95, 0.5 + 0.5 * sin(vWorldPosition.y * 4.8 - uTime * 2.0));

        vec3 baseColor = mix(uPrimaryColor, uRimColor, fresnel * 0.72);
        vec3 finalColor = baseColor + uPrimaryColor * (scanlineFine * 0.34 + scanlineBroad * 0.25) + vec3(fresnel * 0.65);

        float alpha = clamp((0.40 + fresnel * 0.55 + scanlineFine * 0.22) * uOpacity, 0.22, 0.96);
        gl_FragColor = vec4(finalColor, alpha);
      }
    `,
    transparent: true,
    side: THREE.DoubleSide,
    depthWrite: false,
    blending: THREE.AdditiveBlending
  });
}

export function applyRenderStyleToModel(rootObject, styleMode = 'hologram', holoHex = '#00f2fe') {
  rootObject.traverse((child) => {
    if (!child.isMesh || child.name === '__anchor_ring__') return;
    if (!child.userData.originalMaterial) {
      child.userData.originalMaterial = child.material.clone();
    }

    if (styleMode === 'hologram') {
      const mat = createHologramShaderMaterial({ primaryColor: holoHex });
      child.material = mat;
      child.userData.holoMaterial = mat;
      child.castShadow = false;
    } else if (styleMode === 'wireframe') {
      child.material = new THREE.MeshBasicMaterial({
        color: holoHex,
        wireframe: true,
        transparent: true,
        opacity: 0.88
      });
      child.castShadow = false;
    } else {
      child.material = child.userData.originalMaterial;
      child.castShadow = true;
      child.receiveShadow = true;
    }
  });
}
