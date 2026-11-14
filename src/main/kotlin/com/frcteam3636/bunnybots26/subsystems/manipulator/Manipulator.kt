package com.frcteam3636.bunnybots26.subsystems.manipulator

import com.frcteam3636.bunnybots26.robot.Robot
import com.frcteam3636.bunnybots26.subsystems.pivot.Pivot
import com.frcteam3636.bunnybots26.utils.math.volts
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.sequence
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Manipulator : Subsystem {
    private val io: ManipulatorIO =
        when (Robot.model) {
            Robot.Model.SIMULATION -> ManipulatorIOReal()
            Robot.Model.COMPETITION -> ManipulatorIOReal()
        }

    private val inputs = LoggedManipulatorInputs()

    fun intake(): Command = runEnd(
        { io.setVoltage(10.0.volts)},
        {io.setVoltage(0.0.volts)}
    )

    fun outtake(): Command = runEnd(
        { io.setVoltage((-5.0).volts) },
        { io.setVoltage(0.volts)}
    )

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)
    }
}
