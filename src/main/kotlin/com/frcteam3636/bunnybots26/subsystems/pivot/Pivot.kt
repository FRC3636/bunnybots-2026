package com.frcteam3636.bunnybots26.subsystems.pivot

import com.frcteam3636.bunnybots26.robot.Robot
import com.frcteam3636.bunnybots26.utils.math.degrees
import com.frcteam3636.bunnybots26.utils.math.inDegrees
import com.frcteam3636.bunnybots26.utils.math.volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

object Pivot : Subsystem{
    private val io: PivotIO =
        when(Robot.model){
            Robot.Model.SIMULATION -> PivotIOReal()
            Robot.Model.COMPETITION -> PivotIOReal()
        }

    private val inputs = LoggedPivotInputs()

    val atDesiredAngle: Trigger =
        Trigger {
            abs((inputs.pivotAngle - inputs.desiredAngle).inDegrees()) < 3
        }

    fun setPivotPosition(position: Position): Command =
        run{
            io.setPivotAngle(position.angle)
        }

    fun setPivotVoltage(volts: Voltage): Command = runEnd(
        {io.setVoltage(volts)},
        {io.setVoltage(0.volts)}
    )

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("pivot", inputs)
    }

    enum class Position(val angle: Angle){
        Stowed(0.0.degrees),
        Intake(1.0.degrees),
        Score(1.0.degrees),
    }
}



