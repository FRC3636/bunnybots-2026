package com.frcteam3636.swervebase.subsystems.drivetrain.Intake
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.frcteam3636.swervebase.CTREDeviceId
import com.frcteam3636.swervebase.TalonFX
import com.frcteam3636.swervebase.utils.math.*
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.Logger
import org.team9432.annotation.Logged

@Logged
open class IntakeInputs {
    var intakeMotorVelocity = 0.rotationsPerSecond
    var intakeMotorCurrent = Amps.zero()!!
}

interface IntakeIO {
    fun setSpeed(percent: Double)
    fun setVoltage(voltage: Voltage)
    fun updateInputs(inputs: IntakeInputs)
}

class IntakeIOReal : IntakeIO {
    companion object Constants {
        val PID_GAINS = PIDGains(120.0, 0.0, 0.5)
        val PROFILE_CRUISE_VELOCITY = 320.0.degreesPerSecond
        val PROFILE_ACCELERATION = 400.degreesPerSecondPerSecond
        val PROFILE_JERK = 20.0
        val ENCODER_TO_PIVOT_GEAR_RATIO = 32.0 / 12.0
        val MOTOR_TO_ENCODER_GEAR_RATIO = 4.0
        val DISCONTINUITY_POINT = 0.999
        val MAGNET_OFFSET = 0.130859375
        val GRAVITY_COMPENSATION_GAIN = 1.0

        val PIVOT_MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive
        val WHEEL_MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive
    }
    private val intakeMotor = TalonFX(CTREDeviceId.IntakeMotor).apply {
        configurator.apply(TalonFXConfiguration().apply { MotorOutput.Inverted =
            com.frcteam3636.swervebase.subsystems.drivetrain.Intake.IntakeIOReal.Constants.WHEEL_MOTOR_DIRECTION
        })
    }

    override fun setSpeed(percent: Double) {
        intakeMotor.set(percent)
    }

    override fun setVoltage(voltage: Voltage) {
        Logger.recordOutput("Intake Did Voltage", voltage)
        intakeMotor.setVoltage(voltage.inVolts())
    }

    override fun updateInputs(inputs: IntakeInputs) {
        inputs.intakeMotorVelocity = intakeMotor.velocity.value
        inputs.intakeMotorCurrent = intakeMotor.supplyCurrent.value
    }
}