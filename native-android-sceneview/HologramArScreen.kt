package com.holocanvas.ar.ui

import android.net.Uri
import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.unit.dp
import com.google.ar.core.Config
import com.google.ar.core.Frame
import com.google.ar.core.Plane
import com.google.ar.core.TrackingState
import io.github.sceneview.ar.ARScene
import io.github.sceneview.ar.node.AnchorNode
import io.github.sceneview.ar.rememberARCameraNode
import io.github.sceneview.model.ModelInstance
import io.github.sceneview.node.ModelNode
import io.github.sceneview.rememberCollisionSystem
import io.github.sceneview.rememberEngine
import io.github.sceneview.rememberMaterialLoader
import io.github.sceneview.rememberModelLoader
import io.github.sceneview.rememberNodes
import io.github.sceneview.rememberView

/**
 * 原生 Android 端 6DoF 平面识别与空间锚定实现
 * 基于 Google ARCore Android SDK (Apache-2.0) 与 SceneView (Apache-2.0):
 * 1. 开启 HORIZONTAL_AND_VERTICAL 平面识别与 Depth API 深度遮挡
 * 2. 开启 planeRenderer 实时渲染识别出的物理平面多边形网格
 * 3. 拖拽本地 .glb/.gltf 模型松手时调用 frame.hitTest(x, y)，过滤 Plane 命中点并创建硬件级 AnchorNode
 */
@Composable
fun HologramArScreen(
    importedModelUris: List<Uri>,
    onPickLocalModel: () -> Unit
) {
    val engine = rememberEngine()
    val modelLoader = rememberModelLoader(engine)
    val materialLoader = rememberMaterialLoader(engine)
    val cameraNode = rememberARCameraNode(engine)
    val childNodes = rememberNodes()
    val view = rememberView(engine)
    val collisionSystem = rememberCollisionSystem(view)

    var currentFrame by remember { mutableStateOf<Frame?>(null) }
    var planeRendererEnabled by remember { mutableStateOf(true) }
    var detectedPlaneCount by remember { mutableIntStateOf(0) }

    Box(modifier = Modifier.fillMaxSize()) {
        ARScene(
            modifier = Modifier.fillMaxSize(),
            childNodes = childNodes,
            engine = engine,
            view = view,
            modelLoader = modelLoader,
            collisionSystem = collisionSystem,
            cameraNode = cameraNode,
            planeRenderer = planeRendererEnabled,
            sessionConfiguration = { session, config ->
                config.planeFindingMode = Config.PlaneFindingMode.HORIZONTAL_AND_VERTICAL
                config.lightEstimationMode = Config.LightEstimationMode.ENVIRONMENTAL_HDR
                config.instantPlacementMode = Config.InstantPlacementMode.LOCAL_Y_UP
                if (session.isDepthModeSupported(Config.DepthMode.AUTOMATIC)) {
                    config.depthMode = Config.DepthMode.AUTOMATIC
                }
            },
            onSessionUpdated = { session, updatedFrame ->
                currentFrame = updatedFrame
                detectedPlaneCount = session.getAllTrackables(Plane::class.java)
                    .count { it.trackingState == TrackingState.TRACKING }
            }
        )
    }
}

/**
 * 将本地 3D 模型精准锚定在 ARCore 识别出的 Plane 平面上 (零摇摆、姿态严格垂直于平面法线)
 */
fun placeModelOnDetectedPlane(
    frame: Frame?,
    dropX: Float,
    dropY: Float,
    modelInstance: ModelInstance,
    engine: com.google.android.filament.Engine,
    childNodes: MutableList<io.github.sceneview.node.Node>
): Boolean {
    val hitResult = frame?.hitTest(dropX, dropY)?.firstOrNull { hit ->
        val trackable = hit.trackable
        trackable is Plane && trackable.isPoseInPolygon(hit.hitPose) &&
            trackable.trackingState == TrackingState.TRACKING
    } ?: return false

    val anchor = hitResult.createAnchor()
    val anchorNode = AnchorNode(engine = engine, anchor = anchor).apply {
        isEditable = true
        addChildNode(
            ModelNode(
                modelInstance = modelInstance,
                scaleToUnits = 0.65f
            )
        )
    }
    childNodes.add(anchorNode)
    return true
}
