package com.frcteam3636.bunnybots26.subsystems.poseEstimator

import com.frcteam3636.bunnybots26.robot.Robot
import com.frcteam3636.bunnybots26.utils.math.seconds
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger
import org.photonvision.PhotonCamera

object PoseEstimator : Subsystem {
    private val photonVision = PhotonCamera("objectDetection")
    private val io: PoseEstimatorIO =
        when (Robot.model) {
            Robot.Model.SIMULATION -> PoseEstimatorIOSim()
            Robot.Model.COMPETITION -> PoseEstimatorIOReal()
        }

    val inputs = LoggedPoseEstimatorInputs()

    private var hb = 0
    val FIELD_LAYOUT: AprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)


    override fun periodic() {
        io.clearVisionMeasurementData(inputs)
        io.updateInputs(inputs)
        Logger.processInputs("Vision", inputs)

        if(hb++ == 50){
            hb = 0
            photonVision.takeInputSnapshot()
        }

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