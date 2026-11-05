package com.frcteam3636.swervebase.subsystems.elevator

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.swervebase.CTREDeviceId
import com.frcteam3636.swervebase.TalonFX
import com.frcteam3636.swervebase.utils.math.PIDGains
import com.frcteam3636.swervebase.utils.math.amps
import com.frcteam3636.swervebase.utils.math.inRotationsPerSecond
import com.frcteam3636.swervebase.utils.math.inVolts
import com.frcteam3636.swervebase.utils.math.inchesPerSecond
import com.frcteam3636.swervebase.utils.math.meters
import com.frcteam3636.swervebase.utils.math.metersPerSecond
import com.frcteam3636.swervebase.utils.math.pidGains
import com.frcteam3636.swervebase.utils.math.rpm
import com.frcteam3636.swervebase.utils.math.toAngular
import com.frcteam3636.swervebase.utils.math.toLinear
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

@Logged
open class ElevatorInputs() {
    var leftMotorVelocity = 0.metersPerSecond
    var rightMotorVelocity = 0.metersPerSecond
    var leftMotorHeight = 0.meters
    var rightMotorHeight = 0.meters
    var leftMotorCurrent = 0.amps
    var rightMotorCurrent = 0.amps
    var setpoint = 0.meters
}

interface ElevatorIO {
    fun updateInputs(inputs: ElevatorInputs)

    fun setVoltage(voltage: Voltage)

    fun runToHeight(height: Distance)
    fun runToHeightWithOverride(height: Distance, velocity: AngularVelocity, acceleration: AngularAcceleration)

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


        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            leftMotor.position,
            rightMotor.position,
            leftMotor.velocity,
            rightMotor.supplyCurrent,
            leftMotor.supplyCurrent
        )
        leftMotor.optimizeBusUtilization()
        rightMotor.optimizeBusUtilization()

    }

    override fun updateInputs(inputs: ElevatorInputs) {
        inputs.leftMotorVelocity = leftMotor.velocity.value.toLinear(SPOOL_RADIUS)
        inputs.rightMotorVelocity = rightMotor.velocity.value.toLinear(SPOOL_RADIUS)

        inputs.rightMotorCurrent = leftMotor.supplyCurrent.value
        inputs.leftMotorCurrent = leftMotor.supplyCurrent.value

        inputs.leftMotorHeight = leftMotor.position.value.toLinear(SPOOL_RADIUS)
        inputs.rightMotorHeight = rightMotor.position.value.toLinear(SPOOL_RADIUS)

        inputs.setpoint = setPoint
    }

    override fun setVoltage(voltage: Voltage) {
        leftMotor.setVoltage(voltage.inVolts())
        rightMotor.set(voltage.inVolts())
    }

    override fun runToHeight(height: Distance) {
        setPoint = height
        if(height in MIN_HEIGHT..MAX_HEIGHT){ return }

        val desiredMotorAngle = height.toAngular(SPOOL_RADIUS)
        val controlRequest = MotionMagicVoltage(desiredMotorAngle)

        leftMotor.setControl(controlRequest)
    }

    override fun runToHeightWithOverride(
        height: Distance,
        velocity: AngularVelocity,
        acceleration: AngularAcceleration
    ) {
        setPoint = height
        val desiredAngle = height.toAngular(SPOOL_RADIUS)
        val controlRequest = MotionMagicVoltage(desiredAngle)
        leftMotor.setControl(controlRequest)
    }

    override fun setEncoderPosition(position: Distance) {
        leftMotor.setPosition(position.toAngular(SPOOL_RADIUS))
        rightMotor.setPosition(position.toAngular(SPOOL_RADIUS))
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
        val SPOOL_RADIUS = 0.0.meters
        val MAX_HEIGHT = 0.meters
        val MIN_HEIGHT = 0.meters
    }
}


