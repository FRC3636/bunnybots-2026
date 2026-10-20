package com.frcteam3636.swervebase.subsystems.elevator

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.sim.TalonFXSimState
import com.frcteam3636.swervebase.CTREDeviceId
import com.frcteam3636.swervebase.utils.math.PIDGains
import com.frcteam3636.swervebase.utils.math.amps
import com.frcteam3636.swervebase.utils.math.inRotationsPerSecond
import com.frcteam3636.swervebase.utils.math.inches
import com.frcteam3636.swervebase.utils.math.inchesPerSecond
import com.frcteam3636.swervebase.utils.math.meters
import com.frcteam3636.swervebase.utils.math.pidGains
import com.frcteam3636.swervebase.utils.math.rpm
import com.frcteam3636.swervebase.utils.math.toAngular
import com.frcteam3636.swervebase.utils.math.volts
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage

open class ElevatorInputs() {
    var leftMotorVelocity = 0.rpm
    var rightMotorVelocity = 0.rpm

    var leftMotorVoltage = 0.volts
    var rightMotorVoltage = 0.volts

    var leftMotorPosition = 0.meters
    var rightMotorPosition = 0.meters

    var leftMotorCurrent = 0.amps
    var rightMotorCurrent = 0.amps

    var leftMotorSetPoint = 0.meters
    var rightMotorSetPoint = 0.meters
}

interface ElevatorIO {
    fun updateInputs(inputs: ElevatorInputs)

    fun setVoltage(voltage: Voltage)

    fun runToHeight(height: Distance)

    fun setEncoderPosition(position: Distance)

    fun setBrakeMode(enabled: Boolean)
}

class ElevatorIOReal : ElevatorIO {
    var leftMotorConfig = TalonFXConfiguration()
    var rightMotorConfig = TalonFXConfiguration()

    var leftMotor = TalonFX

    
    init{
        leftMotorConfig.apply {
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
            }

            Slot0.apply {
                pidGains = PID_GAINS
            }

            Feedback.apply {
                SensorToMechanismRatio = ROTOR_TO_MECHANISM_GEAR_RATIO
            }

            MotionMagic.apply {
                MotionMagicCruiseVelocity = PROFILE_VELOCITY.inRotationsPerSecond()
                MotionMagicAcceleration = PROFILE_ACCELERATION
                MotionMagicJerk = PROFILE_JERK
            }
        }
    }

    override fun updateInputs(inputs: ElevatorInputs) {
    }

    override fun setVoltage(voltage: Voltage) {
        TODO("Not yet implemented")
    }

    override fun runToHeight(height: Distance) {
        TODO("Not yet implemented")
    }

    override fun setEncoderPosition(position: Distance) {
        TODO("Not yet implemented")
    }

    override fun setBrakeMode(enabled: Boolean) {
        TODO("Not yet implemented")
    }

    companion object Constants{

        private const val ROTOR_TO_MECHANISM_GEAR_RATIO = 8.0
        val SPOOL_RADIUS = 0.77.inches

        //        private val DISTANCE_PER_TURN = Meters.per(Radian).of(SPOOL_RADIUS.meters)
        private val PID_GAINS = PIDGains(30.0, 0.0, 0.0)

        //        private val FF_GAINS = MotorFFGains(0.039214, 1.0233, 0.025904)
//        private const val GRAVITY_GAIN = 0.27592
        private const val PROFILE_ACCELERATION = 50.0
        private const val PROFILE_JERK = 0.0
        private val PROFILE_VELOCITY = 350.inchesPerSecond.toAngular(SPOOL_RADIUS)
    }

}


