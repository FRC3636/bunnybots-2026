package com.frcteam3636.swervebase.subsystems.intake

import com.frcteam3636.swervebase.Robot
import com.frcteam3636.swervebase.subsystems.drivetrain.Intake.IntakeIO
import com.frcteam3636.swervebase.subsystems.drivetrain.Intake.IntakeIOReal
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.Logger
import com.frcteam3636.swervebase.utils.math.degrees
import com.frcteam3636.swervebase.utils.math.volts
import kotlin.math.abs

object Intake : Subsystem {

    enum class Position(val angle: Angle) {
        Deployed(110.degrees),
        Stowed(75.degrees),
        Back(10.degrees),
    }

    private val io: IntakeIO =
        when (Robot.model) {
            Robot.Model.SIMULATION -> IntakeIOReal()
            Robot.Model.COMPETITION -> IntakeIOReal()

        }

    private val inputs = LoggedIntakeInputs()



    fun intake(): Command =
        runEnd(
            { io.setVoltage(10.0.volts) },
            { io.setVoltage(0.volts) }
        )

    fun outtake(): Command = runEnd(
        { io.setVoltage((-5.0).volts) },
        { io.setVoltage(0.volts) },
    )

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)
    }
}
