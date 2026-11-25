package com.frcteam3636.bunnybots26.subsystems.objectPoseEstimator

import com.frcteam3636.bunnybots26.subsystems.poseEstimator.PoseEstimator
import com.frcteam3636.bunnybots26.utils.math.degrees
import com.frcteam3636.bunnybots26.utils.math.inMeters
import com.frcteam3636.bunnybots26.utils.math.inMillimeters
import com.frcteam3636.bunnybots26.utils.math.inRadians
import com.frcteam3636.bunnybots26.utils.math.meters
import com.frcteam3636.bunnybots26.utils.math.millimeters
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import org.littletonrobotics.junction.Logger
import org.photonvision.PhotonCamera
import org.team9432.annotation.Logged
import kotlin.math.tan

@Logged
open class ObjectPoseProviderInputs(){
    /* Targets the camera can see*/
    var targets: Array<TargetObject> = arrayOf()

    var lastSeenTargets: Array<TargetObject> = arrayOf()
    var isConnected: Boolean = false

}

interface ObjectPoseProvider{
    fun updateInputs(inputs: ObjectPoseProviderInputs)
}


class PhotonObjectPoseProvider(name: String, val chassisToCamera: Transform3d): ObjectPoseProvider{
    private val camera = PhotonCamera(name)

    override fun updateInputs(inputs: ObjectPoseProviderInputs) {
        inputs.isConnected = camera.isConnected
        inputs.targets = arrayOf()
        camera.takeInputSnapshot()
        camera.takeOutputSnapshot()
        val result = camera.allUnreadResults.first()
        if(result.hasTargets()) {
            for (target in result.targets) {
                if (target.objDetectConf < 0.9) continue
                target.bestCameraToTarget

                val corners = target.detectedCorners

                val maxY = corners.maxOf { it.y }
                val minY = corners.minOf { it.y }
                val maxX = corners.maxOf { it.x }
                val minX = corners.maxOf { it.x }

                val heightInPixels = maxY - minY
                val widthInPixels = maxX - minX

                val centerY = (maxY + minY) / 2
                val centerX = (maxX + minX) / 2

                val magnification = CONE_HEIGHT.inMeters() / heightInPixels

                Logger.recordOutput("magnification", magnification)

                val xDistance = magnification * (centerY - 320) / tan(target.pitch.degrees.inRadians())
                val yDistance = magnification * (centerY - 320)
                val zDistance = magnification * (centerX - 320)

                val targetOffset = Transform3d(xDistance, yDistance, zDistance, Rotation3d.kZero)

                var camearaPose = Pose3d(PoseEstimator.inputs.estimatedPose) + cameraOffset

                Logger.recordOutput("Cone Pose", camearaPose + targetOffset)



            }
        }
    }

    companion object Constants{
        val cameraOffset: Transform3d = Transform3d(1.0, 0.0, 0.0, Transform3d.kZero.rotation)
        val CONE_HEIGHT = 0.485.meters
        val CONE_HEIGHT_MINUS_BASE = 0.460.meters
        val CONE_WIDTH_MINUS_BASE = 0.189.meters
    }
}


data class TargetObject(
    val pose: Pose3d,
    val confidence: Double,
    val timeStamp: Time,
)

enum class TargetType(val value: Int){
    RedCone(1),
    BlueCone(2),
    WhiteCone(3);

    fun getTarget(id: Int): TargetType{
        return when(id){
            RedCone.value -> TargetType.RedCone
            BlueCone.value -> TargetType.BlueCone
            WhiteCone.value -> TargetType.WhiteCone
            else -> {throw IllegalArgumentException("Invalid TargetType: $id")}
        }

    }
}


