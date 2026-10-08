package com.frcteam3636.swervebase.subsystems.drivetrain.Intake
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.frcteam3636.swervebase.CANcoder
import com.frcteam3636.swervebase.CTREDeviceId
import com.frcteam3636.swervebase.TalonFX
import com.frcteam3636.swervebase.utils.math.PIDGains
import com.frcteam3636.swervebase.utils.math.amps
import com.frcteam3636.swervebase.utils.math.degrees
import com.frcteam3636.swervebase.utils.math.degreesPerSecond
import com.frcteam3636.swervebase.utils.math.degreesPerSecondPerSecond
import com.frcteam3636.swervebase.utils.math.inRadians
import com.frcteam3636.swervebase.utils.math.inRotationsPerSecond
import com.frcteam3636.swervebase.utils.math.inRotationsPerSecondPerSecond
import com.frcteam3636.swervebase.utils.math.inVolts
import com.frcteam3636.swervebase.utils.math.pidGains
import com.frcteam3636.swervebase.utils.math.rotationsPerSecond
import com.frcteam3636.swervebase.utils.math.rotationsPerSecondPerSecond
import com.frcteam3636.swervebase.utils.math.volts
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.Logger
import org.team9432.annotation.Logged
import kotlin.apply
import kotlin.math.sin

@Logged
open class IntakeInputs {
    var intakeMotorVelocity = 0.rotationsPerSecond
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

    private var intakeMotor = TalonFX(CTREDeviceId.IntakeMotor).apply {
        configurator.apply(TalonFXConfiguration().apply {
            Slot0.apply {
                pidGains = PID_GAINS
            }
            MotionMagic.apply {
                MotionMagicCruiseVelocity = PROFILE_CRUISE_VELOCITY.inRotationsPerSecond()
                MotionMagicAcceleration = PROFILE_ACCELERATION.inRotationsPerSecondPerSecond()
                MotionMagicJerk = PROFILE_JERK
            }
            Feedback.apply {
                FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder
                FeedbackRemoteSensorID = CTREDeviceId.IntakePivotEncoder.num
                SensorToMechanismRatio = ENCODER_TO_PIVOT_GEAR_RATIO
                RotorToSensorRatio = MOTOR_TO_ENCODER_GEAR_RATIO
            }
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = PIVOT_MOTOR_DIRECTION
            }
        })
    }

    override fun setSpeed(percent: Double) {
        intakeMotor.set(percent)
    }

    override fun setVoltage(voltage: Voltage) {
        intakeMotor.setVoltage(voltage)
    }

    override fun updateInputs(inputs: IntakeInputs) {
        inputs.intakeMotorVelocity = intakeMotor.encoder.velocity.rpm
    }
}