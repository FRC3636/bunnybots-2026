package com.frcteam3636.bunnybots26.subsystems.poseEstimator

import com.frcteam3636.bunnybots26.robot.Robot
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object PoseEstimator : Subsystem {

    private val io: PoseEstimatorIO =
        when (Robot.model) {
            Robot.Model.SIMULATION -> PoseEstimatorIOSim()
            Robot.Model.COMPETITION -> PoseEstimatorIOReal()
        }

    val inputs = LoggedPoseEstimatorInputs()

    val FIELD_LAYOUT: AprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)

    override fun periodic() {
        io.clearMeasurementData(inputs)
        io.updateInputs(inputs)
        Logger.processInputs("Vision", inputs)
    }

    fun resetPose(pose: Pose2d): Command = Commands.runOnce(
        { io.resetPose(pose) },
    )

    fun resetRotation(rotation: Rotation2d): Command = Commands.runOnce(
        { io.resetRotation(rotation) }
    )

    enum class FieldSide {
        Left,
        Right
    }
}