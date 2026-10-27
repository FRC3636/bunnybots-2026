package com.frcteam3636.bunnybots26.subsystems.intake
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.frcteam3636.bunnybots26.CTREDeviceId
import com.frcteam3636.bunnybots26.TalonFX
import com.frcteam3636.bunnybots26.utils.math.*
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

@Logged
open class IntakeInputs {
    var intakeMotorVelocity = 0.rotationsPerSecond
    var intakeMotorCurrent = Amps.zero()!!
    var intakeMotorVoltage = Volts.zero()!!
}

interface IntakeIO {
    fun setSpeed(percent: Double)
    fun setVoltage(voltage: Voltage)
    fun updateInputs(inputs: IntakeInputs)
}

class IntakeIOReal : IntakeIO {
    companion object Constants {
        val WHEEL_MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive
    }
    private val intakeMotor = TalonFX(CTREDeviceId.IntakeMotor).apply {
        configurator.apply(
            TalonFXConfiguration().apply {
                MotorOutput.Inverted = WHEEL_MOTOR_DIRECTION
            }
        )
    }

    override fun setSpeed(percent: Double) {
        intakeMotor.set(percent)
    }

    override fun setVoltage(voltage: Voltage) {
        intakeMotor.setVoltage(voltage.inVolts())
    }

    override fun updateInputs(inputs: IntakeInputs) {
        inputs.intakeMotorVelocity = intakeMotor.velocity.value
        inputs.intakeMotorCurrent = intakeMotor.supplyCurrent.value
        inputs.intakeMotorVoltage = intakeMotor.supplyVoltage.value
    }
}