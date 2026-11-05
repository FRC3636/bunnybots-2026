package com.frcteam3636.swervebase.subsystems.elevator

import com.frcteam3636.swervebase.Robot
import com.frcteam3636.swervebase.utils.math.amps
import com.frcteam3636.swervebase.utils.math.inches
import com.frcteam3636.swervebase.utils.math.meters
import com.frcteam3636.swervebase.utils.math.volts
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import org.littletonrobotics.junction.Logger

object Elevator : Subsystem {
    private val io : ElevatorIO = when(Robot.model){
        Robot.Model.SIMULATION -> ElevatorIOReal()
        Robot.Model.COMPETITION -> ElevatorIOReal()
    }

    val inputs = LoggedElevatorInputs()

    var position = Position.Stowed
    private var desiredHeight = 0.meters

    val isBlocked get() = inputs.leftMotorCurrent > Constants.CURRENT_LIMIT || inputs.rightMotorCurrent > Constants.CURRENT_LIMIT
    val isAtTarget get() = (inputs.leftMotorHeight- desiredHeight) < Constants.ELEVATOR_TOLERANCE

    private val sysID = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            null,
            null,
            {
                    state -> Logger.recordOutput("SysIdTestState", state.toString())
            }
        ),
        SysIdRoutine.Mechanism(
            io::setVoltage,
            null,
            this
        )
    )

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Elevator", inputs)
    }

    fun getToHeight(position: Position): Command =
        startEnd({
            desiredHeight = position.height
            this.position = position
            io.runToHeight(desiredHeight)
        },{
            if(isBlocked){

            }
        }).until { isAtTarget || isBlocked }

    fun runHoming(): Command =
        runEnd({
            io.setVoltage((-1).volts)
        },{
            io.setVoltage(0.volts)
            io.setEncoderPosition(0.meters)
        }).until { isBlocked }

    fun coast(): Command = Commands.startEnd({
        io.setBrakeMode(false)
    }, {
        io.setBrakeMode(true)
    })

    fun sysIdQuasistatic(direction: Direction): Command = sysID.quasistatic(direction)

    fun sysIdDynamic(direction: Direction): Command = sysID.dynamic(direction)
}

enum class Position(val height: Distance){
    Stowed(0.meters)
}

object Constants{
    val ELEVATOR_TOLERANCE = 0.75.inches
    val CURRENT_LIMIT = 1.9.amps
}