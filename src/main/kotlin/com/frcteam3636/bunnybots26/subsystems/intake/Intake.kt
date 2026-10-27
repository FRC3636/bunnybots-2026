package com.frcteam3636.bunnybots26.subsystems.intake

import com.frcteam3636.bunnybots26.Robot
import com.frcteam3636.bunnybots26.subsystems.drivetrain.intake.LoggedIntakeInputs
import com.frcteam3636.bunnybots26.utils.math.volts
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Intake : Subsystem {

    private val io: IntakeIO =
        when (Robot.model) {
            Robot.Model.SIMULATION -> IntakeIOReal()
            Robot.Model.COMPETITION -> IntakeIOReal()
        }

    private val inputs = LoggedIntakeInputs()

    fun intake(): Command = runEnd(
        { io.setVoltage(10.0.volts) },
        { io.setVoltage(0.volts) }
    )

    fun outtake(): Command = runEnd(
        { io.setVoltage((-5.0).volts) },
        { io.setVoltage(0.volts) }
    )

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)
    }
}
