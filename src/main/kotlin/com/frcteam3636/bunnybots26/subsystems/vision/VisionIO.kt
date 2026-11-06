package com.frcteam3636.bunnybots26.subsystems.vision
import com.frcteam3636.bunnybots26.subsystems.drivetrain.Drivetrain
import com.frcteam3636.bunnybots26.subsystems.vision.Vision.FIELD_LAYOUT
import com.frcteam3636.bunnybots26.utils.math.TAU
import com.frcteam3636.bunnybots26.utils.math.degrees
import com.frcteam3636.bunnybots26.utils.math.meters
import edu.wpi.first.math.geometry.*
import org.littletonrobotics.junction.Logger
import org.team9432.annotation.Logged
import kotlin.math.abs

@Logged
open class VisionInputs {
    var acceptedPoses = arrayOf<Pose2d>()
    var rejectedPoses = arrayOf<Pose2d>()
    var observedTags = intArrayOf()
    var observedTagPoses = arrayOf<Pose3d>()
    var allProvidersConnected = false
    var poseSupplierInitialized = false
}

interface VisionIO {
    fun clearMeasurementData(inputs: VisionInputs)
    fun registerPoseSupplier(supplier: () -> Pose2d, enabledSupplier: () -> Boolean)
    fun updateInputs(inputs: VisionInputs)
    fun drainAcceptedMeasurements(): List<AbsolutePoseMeasurement>
}

class VisionIOReal : VisionIO {

    private val providers = mapOf(
        "Left" to
            LimelightPoseProvider("limelight-left", { poseSupplier().rotation }, { Drivetrain.inputs.gyroVelocity }, { Drivetrain.inputs.gyroConnected }, isLL4 = true),
        "Right" to
            LimelightPoseProvider("limelight-right",{ poseSupplier().rotation }, { Drivetrain.inputs.gyroVelocity }, { Drivetrain.inputs.gyroConnected }, isLL4 = true),
        "LeftTrig" to
            TrigSolverPoseProvider("limelight-left",LEFT_CAMERA_TRANSFORM,{ poseSupplier().rotation },{ Drivetrain.inputs.gyroVelocity }, isLL4 = true),
        "RightTrig" to
            TrigSolverPoseProvider("limelight-right",RIGHT_CAMERA_TRANSFORM,{ poseSupplier().rotation },{ Drivetrain.inputs.gyroVelocity }, isLL4 = true),
    ).mapValues { Pair(it.value, LoggedAbsolutePoseProviderInputs()) }

    private lateinit var poseSupplier: () -> Pose2d
    private lateinit var enabledSupplier: () -> Boolean

    private val acceptedMeasurements = mutableListOf<AbsolutePoseMeasurement>()

    override fun clearMeasurementData(inputs: VisionInputs) {
        inputs.acceptedPoses = arrayOf()
        inputs.rejectedPoses = arrayOf()
        inputs.observedTags = intArrayOf()
        inputs.observedTagPoses = arrayOf()
        acceptedMeasurements.clear()
    }

    override fun registerPoseSupplier(supplier: () -> Pose2d, enabledSupplier: () -> Boolean) {
        this.poseSupplier = supplier
        this.enabledSupplier = enabledSupplier
    }

    override fun updateInputs(inputs: VisionInputs) {
        inputs.allProvidersConnected = providers.values.all { it.second.connected }
        inputs.poseSupplierInitialized = ::poseSupplier.isInitialized

        for ((name, pair) in providers) {
            val (io, ins) = pair
            io.updateInputs(ins)
            Logger.processInputs("Vision/$name", ins)
            inputs.observedTags += ins.observedTags
            for (m in ins.measurements) {
                when {
                    m.isLowQuality -> reject(inputs, m)
                    m.pose.x < 0.0 || m.pose.y < 0.0 -> reject(inputs, m)
                    m.pose.x > FIELD_LAYOUT.fieldLength || m.pose.y > FIELD_LAYOUT.fieldWidth -> reject(inputs, m)
                    !enabledSupplier() && abs((m.pose.rotation.radians - poseSupplier().rotation.radians).mod(TAU)) > 0.0872 -> reject(inputs, m)
                    else -> accept(inputs, m)
                }
            }
        }

        inputs.observedTagPoses =
            FIELD_LAYOUT.tags
                .filter { it.ID in inputs.observedTags }
                .map { it.pose }
                .toTypedArray()
    }

    override fun drainAcceptedMeasurements(): List<AbsolutePoseMeasurement> {
        val out = acceptedMeasurements.toList()
        acceptedMeasurements.clear()
        return out
    }

    private fun accept(inputs: VisionInputs, m: AbsolutePoseMeasurement) {
        acceptedMeasurements.add(m)
        inputs.acceptedPoses += m.pose
    }

    private fun reject(inputs: VisionInputs, m: AbsolutePoseMeasurement) {
        inputs.rejectedPoses += m.pose
    }

    companion object Constants {
        private val LEFT_CAMERA_TRANSFORM: Transform3d = Transform3d(Translation3d(0.meters,0.meters,0.meters), Rotation3d(Rotation2d(0.degrees)))
        private val RIGHT_CAMERA_TRANSFORM: Transform3d = Transform3d(Translation3d(0.meters,0.meters,0.meters), Rotation3d(Rotation2d(0.degrees)))
    }
}

class VisionIOSim : VisionIO {

    override fun clearMeasurementData(inputs: VisionInputs) {
        TODO("Not yet implemented")
    }

    override fun registerPoseSupplier(supplier: () -> Pose2d, enabledSupplier: () -> Boolean) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: VisionInputs) {
        TODO("Not yet implemented")
    }

    override fun drainAcceptedMeasurements(): List<AbsolutePoseMeasurement> {
        TODO("Not yet implemented")
    }

}