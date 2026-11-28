package com.frcteam3636.bunnybots26.subsystems.objectPoseEstimator

import com.frcteam3636.bunnybots26.subsystems.poseEstimator.PoseEstimator
import com.frcteam3636.bunnybots26.utils.autos.isRedAlliance
import com.frcteam3636.bunnybots26.utils.math.degrees
import com.frcteam3636.bunnybots26.utils.math.inDegrees
import com.frcteam3636.bunnybots26.utils.math.inMeters
import com.frcteam3636.bunnybots26.utils.math.inSeconds
import com.frcteam3636.bunnybots26.utils.math.meters
import com.frcteam3636.bunnybots26.utils.math.radians
import com.frcteam3636.bunnybots26.utils.math.seconds
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Time
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import org.ejml.data.Matrix
import org.littletonrobotics.junction.Logger
import org.photonvision.PhotonCamera
import org.team9432.annotation.Logged
import java.nio.ByteBuffer
import kotlin.math.abs
import kotlin.math.atan2


@Logged
open class ObjectPoseProviderInputs(){
    var targets: Array<TargetObject> = arrayOf()
    var isConnected: Boolean = false
}

interface ObjectPoseProvider{
    fun updateInputs(inputs: ObjectPoseProviderInputs)
}


class PhotonObjectPoseProvider(name: String, val cameraOffset: Transform3d): ObjectPoseProvider{
    val camera: PhotonCamera = PhotonCamera(name)

    val fx: Double
    val fy: Double
    val cx: Double
    val cy: Double


    init{
        if(camera.cameraMatrix.isEmpty){
            fx = 0.0
            fy = 0.0
            cx = 0.0
            cy = 0.0
        }else{
            val matrix = camera.cameraMatrix.get()
            fx = matrix[0,0]
            fy = matrix[1,1]
            cx = matrix[0,2]
            cy = matrix[1,2]
        }
    }

    override fun updateInputs(inputs: ObjectPoseProviderInputs) {
        inputs.isConnected = camera.isConnected
        inputs.targets = arrayOf()
        val measurement: MutableList<TargetObject> = mutableListOf()
        val result = camera.allUnreadResults.lastOrNull() ?: return
        val robotPose = PoseEstimator.inputs.estimatedPose

        if(result.hasTargets()) {
            Logger.recordOutput("Targets", result.targets.size)
            for (target in result.targets) {
                val corners = target.minAreaRectCorners

                if(corners.isEmpty()) continue

                if(TargetType.isOpposingTarget(target.objDetectId) && abs(target.yaw) > 10.0) continue
                //The yaw check will be removed once the new model is trained


                //math from here https://sites.ecse.rpi.edu/~qji/CV/perspective_geometry2.pdf
                val maxY = corners.maxOf { it.y }
                val minY = corners.minOf { it.y }
                val maxX = corners.maxOf { it.x }
                val minX = corners.maxOf { it.x }

                val heightInPixels = maxY - minY

                if(heightInPixels <= 0) continue

                val normalizedCenterY = (maxY + minY) / 2 - cy
                val normalizedCenterX = (maxX + minX) / 2 - cx

                val xCameraRelative = fy * CONE_HEIGHT.inMeters() / heightInPixels
                val yCameraRelative = normalizedCenterX * xCameraRelative / -fx
                val zCameraRelative = normalizedCenterY * xCameraRelative / -fy

                val robotRelativePose = cameraOffset
                    .plus(Transform3d(
                        xCameraRelative,
                        yCameraRelative,
                        zCameraRelative,
                        Rotation3d()
                    )
                )


//                if(TargetType.isConeTower(target.objDetectId)){
                if(target.pitch < 10){
                    val nearestPose = ConeTower.getNearestWithinFov(robotPose) ?: ConeTower.getNearest(robotPose)
                    Logger.recordOutput("Predicted Robot Pose", Pose2d(nearestPose.pose.minus(robotPose.translation), robotPose.rotation))
                }


                val targetPose = Pose3d(PoseEstimator.inputs.estimatedPose)
                    .transformBy(robotRelativePose)

                measurement.add(TargetObject(
                    targetPose,
                    target.objDetectConf,
                    TargetType.getTarget(target.objDetectId),
                    result.timestampSeconds.seconds
                ))
            }
        }
        inputs.targets = measurement.toTypedArray()
    }

    companion object Constants{
        val CONE_HEIGHT = 0.485.meters
        val FOV = 120.degrees// Filler for now, put a real number in later
    }
}


data class TargetObject(
    val pose: Pose3d,
    val confidence: Float,
    val type: TargetType,
    val timeStamp: Time,
): StructSerializable{
    companion object {
        @JvmField
        @Suppress("unused")

        val struct = TargetObjectStruct()
    }
}

enum class TargetType(val id: Int){
    RedCone(1),
    BlueCone(0),
    WhiteCone(2),
    ConeTower(3),
    UnknownID(4);

    companion object {
        fun getTarget(id: Int): TargetType{
            return entries.firstOrNull{it.id == id} ?: UnknownID.also {
                Logger.recordOutput("Unknown Target ID", id)
            }
        }

        fun isOpposingTarget(id: Int): Boolean{
            return if(isRedAlliance()){
                (id == BlueCone.id)
            }else{
                (id == RedCone.id)
            }
        }

        fun isConeTower(id: Int): Boolean{
            return id == ConeTower.id
        }
    }
}


enum class ConeTower(val pose: Translation2d){
    CenterBlue(Translation2d((-7.125).meters,0.meters)),
    LeftBlue(Translation2d((-4.381499).meters, 3.556001.meters)),
    RightBlue(Translation2d((-4.381499).meters,(-3.556001).meters)),

    MiddleLeft(Translation2d(0.meters,3.365499.meters)),
    MiddleRight(Translation2d(0.meters, (-3.365499).meters)),

    CenterRed(Translation2d(7.125.meters,0.meters)),
    LeftRed(Translation2d(4.381499.meters, 3.556001.meters)),
    RightLeft(Translation2d(4.381499.meters, (-3.556001).meters));

    companion object{
        fun getNearestWithinFov(robotPose: Pose2d): ConeTower? {
            return entries.filter {
                val angleToPose = atan2(it.pose.y - robotPose.y, it.pose.x - robotPose.x).radians.inDegrees();
                val robotAngle = robotPose.rotation.degrees

                val fovOffset = PhotonObjectPoseProvider.FOV.inDegrees() / 2.0

                val deltaAngle = abs((angleToPose - robotAngle + 180).mod(360.0) - 180)

                deltaAngle < fovOffset
            }.minByOrNull {robotPose.translation.getDistance(it.pose)}
        }
        fun getNearest(robotPose: Pose2d): ConeTower{
            return entries.minBy{ robotPose.translation.getDistance(it.pose)}
        }
    }
}

class TargetObjectStruct : Struct<TargetObject> {

    override fun getTypeClass(): Class<TargetObject> = TargetObject::class.java

    override fun getTypeName(): String = "TargetObject"

    override fun getTypeString(): String = "struct:TargetObject"

    override fun getSize(): Int =
        Pose3d.struct.size + Struct.kSizeFloat + Struct.kSizeInt32 + Struct.kSizeDouble

    override fun getSchema(): String =
        "Pose3d pose;float confidence;enum {blue_cone=0,red_cone=1,white_cone=2,tower=3} int32 type;double timeStamp"

    override fun unpack(bb: ByteBuffer): TargetObject {
        return TargetObject(
            pose = Pose3d.struct.unpack(bb),
            confidence = bb.float,
            type = TargetType.getTarget(bb.int),
            timeStamp = bb.double.seconds
        )
    }
    override fun pack(bb: ByteBuffer, value: TargetObject) {
        Pose3d.struct.pack(bb, value.pose)
        bb.putFloat(value.confidence)
        bb.putInt(value.type.id)
        bb.putDouble(value.timeStamp.inSeconds())
    }
}



