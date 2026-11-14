package com.frcteam3636.bunnybots26.subsystems.manipulator
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.frcteam3636.bunnybots26.CTREDeviceId
import com.frcteam3636.bunnybots26.TalonFX
import com.frcteam3636.bunnybots26.utils.math.*
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

@Logged
open class ManipulatorInputs {
    var manipulatorMotorVelocity = 0.rotationsPerSecond
    var manipulatorMotorCurrent = 0.amps
    var manipulatorMotorVoltage = 0.volts
}

interface ManipulatorIO {
    fun setSpeed(percent: Double)
    fun setVoltage(voltage: Voltage)
    fun updateInputs(inputs: ManipulatorInputs)
}

class ManipulatorIOReal : ManipulatorIO {
    private val manipulatorMotor = TalonFX(CTREDeviceId.IntakeMotor).apply {
        configurator.apply(
            TalonFXConfiguration().apply {
                MotorOutput.Inverted = WHEEL_MOTOR_DIRECTION
            }
        )
    }

    override fun setSpeed(percent: Double) {
        manipulatorMotor.set(percent)
    }

    override fun setVoltage(voltage: Voltage) {
        manipulatorMotor.setVoltage(voltage.inVolts())
    }

    override fun updateInputs(inputs: ManipulatorInputs) {
        inputs.manipulatorMotorVoltage = manipulatorMotor.supplyVoltage.value
        inputs.manipulatorMotorCurrent = manipulatorMotor.supplyCurrent.value
        inputs.manipulatorMotorVelocity = manipulatorMotor.velocity.value
    }

    companion object Constants {
        val WHEEL_MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive
    }
}