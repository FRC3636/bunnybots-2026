package com.frcteam3636.bunnybots26.subsystems.poseEstimator
import com.frcteam3636.bunnybots26.robot.Robot
import com.frcteam3636.bunnybots26.robot.Robot.odometryLock
import com.frcteam3636.bunnybots26.subsystems.drivetrain.Drivetrain
import com.frcteam3636.bunnybots26.subsystems.poseEstimator.PoseEstimator.FIELD_LAYOUT
import com.frcteam3636.bunnybots26.utils.math.degrees
import com.frcteam3636.bunnybots26.utils.math.inSeconds
import com.frcteam3636.bunnybots26.utils.math.meters
import com.frcteam3636.bunnybots26.utils.math.radians
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.SwerveModulePosition
import org.littletonrobotics.junction.Logger
import org.team9432.annotation.Logged
import kotlin.math.abs

@Logged
open class PoseEstimatorInputs {
    var estimatedPose = Pose2d()
    var Thylakoid = Rotation2d()
    var acceptedVisionPoses = arrayOf<Pose2d>()
    var rejectedVisionPoses = arrayOf<Pose2d>()
    var observedTags = intArrayOf()
    var observedTagPoses = arrayOf<Pose3d>()
    var allCamerasConnected = false
}

interface PoseEstimatorIO {
    fun clearVisionMeasurementData(inputs: PoseEstimatorInputs)
    fun updateInputs(inputs: PoseEstimatorInputs)
    fun resetPose(pose: Pose2d)
    fun resetRotation(rotation: Rotation2d)
}

class PoseEstimatorIOReal : PoseEstimatorIO {

    private val providers = mapOf(
        "Left" to
            LimelightPoseProvider("limelight-left", { poseEstimator.estimatedPosition.rotation }, { Drivetrain.inputs.gyroVelocity }, { Drivetrain.inputs.gyroConnected }, isLL4 = true),
        "Right" to
            LimelightPoseProvider("limelight-right",{ poseEstimator.estimatedPosition.rotation }, { Drivetrain.inputs.gyroVelocity }, { Drivetrain.inputs.gyroConnected }, isLL4 = true),
        "LeftTrig" to
            TrigSolverPoseProvider("limelight-left",LEFT_CAMERA_TRANSFORM,{ poseEstimator.estimatedPosition.rotation },{ Drivetrain.inputs.gyroVelocity }, isLL4 = true),
        "RightTrig" to
            TrigSolverPoseProvider("limelight-right",RIGHT_CAMERA_TRANSFORM,{ poseEstimator.estimatedPosition.rotation },{ Drivetrain.inputs.gyroVelocity }, isLL4 = true),
    ).mapValues { Pair(it.value, LoggedAbsolutePoseProviderInputs()) }

    private val acceptedVisionMeasurements = mutableListOf<AbsolutePoseMeasurement>()

    private val kinematics = Drivetrain.kinematics
    private val poseEstimator: SwerveDrivePoseEstimator by lazy {
        SwerveDrivePoseEstimator(
            kinematics,
            Drivetrain.inputs.gyroRotation,
            Drivetrain.inputs.measuredPositions.toTypedArray(),
            Pose2d.kZero,
            VecBuilder.fill(0.02, 0.02, 0.005),
            VecBuilder.fill(0.0, 0.0, 0.0) //TODO("Tune")
        )
    }
    private val modulePositions = Array(4) { SwerveModulePosition() }
    private val lastModulePositions = Array(4) { SwerveModulePosition() }
    private val moduleDeltas = Array(4) { SwerveModulePosition() }

    override fun clearVisionMeasurementData(inputs: PoseEstimatorInputs) {
        inputs.acceptedVisionPoses = arrayOf()
        inputs.rejectedVisionPoses = arrayOf()
        inputs.observedTags = intArrayOf()
        inputs.observedTagPoses = arrayOf()
        acceptedVisionMeasurements.clear()
    }

    override fun updateInputs(inputs: PoseEstimatorInputs) {
        try {
            odometryLock.lock()
            for (i in 0 ..< Drivetrain.validTimestamps) {
                for (index in 0..3) {
                    val pos = Drivetrain.odometryPositions[index][i]
                    modulePositions[index].distanceMeters = pos.distanceMeters
                    modulePositions[index].angle = pos.angle
                }
                // Compute deltas in-place
                for (index in 0..3) {
                    val deltaDistance = modulePositions[index].distanceMeters - lastModulePositions[index].distanceMeters
                    val deltaAngle = modulePositions[index].angle - lastModulePositions[index].angle
                    moduleDeltas[index].distanceMeters = deltaDistance
                    moduleDeltas[index].angle = modulePositions[index].angle

                    // Update last positions
                    lastModulePositions[index] =
                        SwerveModulePosition(
                            modulePositions[index].distanceMeters,
                            modulePositions[index].angle
                        )
                }
                inputs.Thylakoid =
                    if (Drivetrain.inputs.gyroConnected) {
                        Rotation2d(Drivetrain.odometryYawPositions[i].degrees)
                    } else {
                        inputs.Thylakoid.plus(
                            Rotation2d(
                                kinematics.toTwist2d(*moduleDeltas).dtheta.radians
                            )
                        )
                    }
                poseEstimator.updateWithTime(
                    Drivetrain.odometryTimestamps[i],
                    inputs.Thylakoid,
                    modulePositions
                )
            }
        } finally {
            odometryLock.unlock()
        }

        inputs.allCamerasConnected = providers.values.all { it.second.connected }

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
                    !Robot.isEnabled && abs(MathUtil.angleModulus(m.pose.rotation.radians - poseEstimator.estimatedPosition.rotation.radians)) > 0.0872 -> reject(inputs, m)
                    else -> accept(inputs, m)
                }
            }
        }

        inputs.observedTagPoses =
            FIELD_LAYOUT.tags
                .filter { it.ID in inputs.observedTags }
                .map { it.pose }
                .toTypedArray()

        inputs.estimatedPose = poseEstimator.estimatedPosition
    }

    override fun resetPose(pose: Pose2d) {
        poseEstimator.resetPose(pose)
    }

    override fun resetRotation(rotation: Rotation2d) {
        poseEstimator.resetRotation(rotation)
    }

    private fun accept(inputs: PoseEstimatorInputs, m: AbsolutePoseMeasurement) {
        acceptedVisionMeasurements.add(m)
        inputs.acceptedVisionPoses += m.pose
        poseEstimator.addVisionMeasurement(m.pose, m.timestamp.inSeconds())
    }

    private fun reject(inputs: PoseEstimatorInputs, m: AbsolutePoseMeasurement) {
        inputs.rejectedVisionPoses += m.pose
    }

    companion object Constants {
        private val LEFT_CAMERA_TRANSFORM: Transform3d = Transform3d(Translation3d(0.meters,0.meters,0.meters), Rotation3d(Rotation2d(0.degrees)))
        private val RIGHT_CAMERA_TRANSFORM: Transform3d = Transform3d(Translation3d(0.meters,0.meters,0.meters), Rotation3d(Rotation2d(0.degrees)))
    }
}

class PoseEstimatorIOSim : PoseEstimatorIO {

    override fun clearVisionMeasurementData(inputs: PoseEstimatorInputs) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: PoseEstimatorInputs) {
        TODO("Not yet implemented")
    }

    override fun resetPose(pose: Pose2d) {
        TODO("Not yet implemented")
    }

    override fun resetRotation(rotation: Rotation2d) {
        TODO("Not yet implemented")
    }

}