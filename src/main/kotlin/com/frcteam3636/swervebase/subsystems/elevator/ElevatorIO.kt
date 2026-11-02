package com.frcteam3636.swervebase.subsystems.elevator

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.sim.TalonFXSimState
import com.frcteam3636.swervebase.CTREDeviceId
import com.frcteam3636.swervebase.TalonFX
import com.frcteam3636.swervebase.utils.math.PIDGains
import com.frcteam3636.swervebase.utils.math.amps
import com.frcteam3636.swervebase.utils.math.degrees
import com.frcteam3636.swervebase.utils.math.inRotationsPerSecond
import com.frcteam3636.swervebase.utils.math.inVolts
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

    var leftMotorPosition = 0.degrees
    var rightMotorPosition = 0.degrees

    var leftMotorCurrent = 0.amps
    var rightMotorCurrent = 0.amps

    var setpoint = 0.meters
}

interface ElevatorIO {
    fun updateInputs(inputs: ElevatorInputs)

    fun setVoltage(voltage: Voltage)

    fun runToHeight(height: Distance)

    fun setEncoderPosition(position: Distance)

    fun setBrakeMode(enabled: Boolean)
}

class ElevatorIOReal : ElevatorIO {
    var motorConfig = TalonFXConfiguration().apply {
        Slot0.apply {
            pidGains = PID_GAINS
        }

        Feedback.apply {
            SensorToMechanismRatio = SensorToMechanismRatio
        }

        MotionMagic.apply {
            MotionMagicCruiseVelocity = PROFILE_VELOCITY.inRotationsPerSecond()
            MotionMagicJerk = PROFILE_JERK
            MotionMagicAcceleration = PROFILE_ACCELERATION
        }

        MotorOutput.apply {
            Inverted = InvertedValue.Clockwise_Positive
            NeutralMode = NeutralModeValue.Brake
        }

        CurrentLimits.apply {
            SupplyCurrentLimit = 40.0
            SupplyCurrentLowerLimit = 30.0
            SupplyCurrentLowerTime = 1.0
        }

    }
    var setPoint = 0.0.meters
    var leftMotor = TalonFX(CTREDeviceId.LeftElevatorMotor)
    var rightMotor = TalonFX(CTREDeviceId.RightElevatorMotor)

    init{
        leftMotor.configurator.apply(motorConfig)
        rightMotor.setControl(Follower(CTREDeviceId.LeftElevatorMotor.num, MotorAlignmentValue.Opposed))
    }

    override fun updateInputs(inputs: ElevatorInputs) {
        inputs.leftMotorVelocity = leftMotor.velocity.value
        inputs.rightMotorVelocity = rightMotor.velocity.value

        inputs.leftMotorVoltage = leftMotor.supplyVoltage.value
        inputs.rightMotorVoltage = rightMotor.supplyVoltage.value

        inputs.rightMotorCurrent = leftMotor.supplyCurrent.value
        inputs.leftMotorCurrent = leftMotor.supplyCurrent.value

        inputs.leftMotorPosition = leftMotor.position.value
        inputs.rightMotorPosition = rightMotor.position.value

        inputs.setpoint = setPoint
    }

    override fun setVoltage(voltage: Voltage) {
        leftMotor.setVoltage(voltage.inVolts())
    }

    override fun runToHeight(height: Distance) {
        TODO("Not yet implemented")
    }

    override fun setEncoderPosition(position: Distance) {
        TODO("Not yet implemented")
    }

    override fun setBrakeMode(enabled: Boolean) {
        if(enabled){
            leftMotor.setNeutralMode(NeutralModeValue.Brake)
            rightMotor.setNeutralMode(NeutralModeValue.Brake)
        }else{
            leftMotor.setNeutralMode(NeutralModeValue.Coast)
            rightMotor.setNeutralMode(NeutralModeValue.Coast)
        }
    }

    companion object Constants{
        val PROFILE_ACCELERATION = 50.0
        val PROFILE_JERK = 0.0
        private val PROFILE_VELOCITY = 350.inchesPerSecond.toAngular(1.meters)
        val PID_GAINS = PIDGains(5.0, 0.0, 0.0)
        val ROTOR_TO_MECHANISM_GEAR_RATIO = 3
        val SPOOL_RADIUS = 0.0
    }
}


