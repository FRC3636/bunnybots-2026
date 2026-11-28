package com.frcteam3636.bunnybots26.subsystems.poseEstimator

import com.frcteam3636.bunnybots26.robot.Robot
import com.frcteam3636.bunnybots26.subsystems.objectPoseEstimator.LoggedObjectPoseProviderInputs
import com.frcteam3636.bunnybots26.subsystems.objectPoseEstimator.PhotonObjectPoseProvider
import com.frcteam3636.bunnybots26.utils.math.degrees
import com.frcteam3636.bunnybots26.utils.math.inches
import com.frcteam3636.bunnybots26.utils.math.seconds
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger
import org.photonvision.PhotonCamera

object PoseEstimator : Subsystem {
    private val io: PoseEstimatorIO =
        when (Robot.model) {
            Robot.Model.SIMULATION -> PoseEstimatorIOSim()
            Robot.Model.COMPETITION -> PoseEstimatorIOReal()
        }

    val photonCamera = PhotonObjectPoseProvider("objectDetection", Transform3d(0.0.inches,11.5.inches, 26.5.inches, Rotation3d()))
    val inputs = LoggedPoseEstimatorInputs()
    val photonVisionInputs = LoggedObjectPoseProviderInputs()
    val FIELD_LAYOUT: AprilTagFieldLayout = AprilTagFieldLayout(Filesystem.getDeployDirectory().path + "/2026-bunnybots-v1.json")


    override fun periodic() {
        io.clearVisionMeasurementData(inputs)
        io.updateInputs(inputs)
        photonCamera.updateInputs(photonVisionInputs)

        Logger.processInputs("Vision", inputs)
        Logger.processInputs("Object Detection", photonVisionInputs)

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