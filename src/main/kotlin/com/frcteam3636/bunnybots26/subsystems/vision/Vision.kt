package com.frcteam3636.bunnybots26.subsystems.vision

import com.frcteam3636.bunnybots26.robot.Robot
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Vision : Subsystem {

    private val io: VisionIO =
        when (Robot.model) {
            Robot.Model.SIMULATION -> VisionIOSim()
            Robot.Model.COMPETITION -> VisionIOReal()
        }

    val inputs = LoggedVisionInputs()

    val FIELD_LAYOUT: AprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)

    override fun periodic() {
        if (!inputs.poseSupplierInitialized) return
        io.clearMeasurementData(inputs)
        io.updateInputs(inputs)
        Logger.processInputs("Vision", inputs)
    }

    /** Consumed by Drivetrain once per loop. */
    fun drainAcceptedMeasurements(): List<AbsolutePoseMeasurement> {
        return io.drainAcceptedMeasurements()
    }

}