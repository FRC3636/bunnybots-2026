package com.frcteam3636.bunnybots26.subsystems.pivot
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.frcteam3636.bunnybots26.CANcoder
import com.frcteam3636.bunnybots26.CTREDeviceId
import com.frcteam3636.bunnybots26.TalonFX
import com.frcteam3636.bunnybots26.utils.math.*
import com.frcteam3636.bunnybots26.utils.math.amps
import com.frcteam3636.bunnybots26.utils.math.degrees
import com.frcteam3636.bunnybots26.utils.math.radians
import com.frcteam3636.bunnybots26.utils.math.volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.Logger
import org.team9432.annotation.Logged
import kotlin.math.sin


@Logged
open class PivotInputs{
    var pivotAngle = 0.degrees
    var desiredAngle = 0.radians
    var voltage = 0.volts
    var current = 0.amps
}

interface PivotIO{
    fun setSpeed(percentage: Double)
    fun setVoltage(volts: Voltage)
    fun updateInputs(inputs: PivotInputs)
    fun setPivotAngle(angle: Angle)
    fun zeroEncoder()
}

class PivotIOReal : PivotIO{
    var setPoint = 0.0.degrees
    private val positionControl = MotionMagicVoltage(0.0)

    private val pivotMotor = TalonFX(CTREDeviceId.PivotEncoder).apply {
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
                FeedbackRemoteSensorID = CTREDeviceId.PivotEncoder.num
                SensorToMechanismRatio = ENCODER_TO_PIVOT_GEAR_RATIO
                RotorToSensorRatio = MOTOR_TO_ENCODER_GEAR_RATIO
            }

            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = PIVOT_MOTOR_DIRECTION
            }

        })
    }

    private val encoder = CANcoder(CTREDeviceId.PivotEncoder).apply {
        configurator.apply(
            CANcoderConfiguration().apply {
                MagnetSensor.apply {
                    SensorDirection = SensorDirectionValue.Clockwise_Positive
                    AbsoluteSensorDiscontinuityPoint = DISCONTINUITY_POINT
                    MagnetOffset = MAGNET_OFFSET
                }
            }

        )
        setPosition(0.0)
    }
    override fun setSpeed(percentage: Double) {
       pivotMotor.set(percentage)
    }

    override fun setVoltage(volts: Voltage) {
        pivotMotor.setVoltage(volts.inVolts())
    }

    override fun updateInputs(inputs: PivotInputs) {
        inputs.voltage = pivotMotor.supplyVoltage.value
        inputs.current = pivotMotor.supplyCurrent.value
        inputs.desiredAngle = setPoint
        inputs.pivotAngle = pivotMotor.position.value
    }

    override fun setPivotAngle(angle: Angle) {
        if(angle in PIVOT_MIN..PIVOT_MAX){
            setPoint = angle
            Logger.recordOutput("Pivot Setpoint", angle)
            pivotMotor.setControl(positionControl.withPosition(angle)
                .withFeedForward(sin(pivotMotor.position.value.inRadians()) * GRAVITY_COMPENSATION_GAIN))
        }
    }

    override fun zeroEncoder() {
        encoder.setPosition(0.0.degrees)
    }


    companion object Constants {
        val PID_GAINS = PIDGains(5.0,0.0,0.0)
        val PROFILE_CRUISE_VELOCITY = 320.0.degreesPerSecond
        val PROFILE_ACCELERATION = 400.degreesPerSecondPerSecond
        val PROFILE_JERK = 20.0
        val PIVOT_MOTOR_DIRECTION = InvertedValue.Clockwise_Positive
        val PIVOT_MAX = 2.0.degrees
        val PIVOT_MIN = 0.0.degrees
        val GRAVITY_COMPENSATION_GAIN = 1.0
        val ENCODER_TO_PIVOT_GEAR_RATIO = 1.0
        val MOTOR_TO_ENCODER_GEAR_RATIO = 1.0
        val DISCONTINUITY_POINT = 0.999
        val MAGNET_OFFSET = 0.0
    }
}

