package com.frcteam3636.bunnybots26.subsystems.drivetrain

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.MountPoseConfigs
import com.frcteam3636.bunnybots26.CTREDeviceId
import com.frcteam3636.bunnybots26.Pigeon2
import com.frcteam3636.bunnybots26.robot.Robot
import com.frcteam3636.bunnybots26.subsystems.drivetrain.Drivetrain.Constants.BRAKE_POSITION
import com.frcteam3636.bunnybots26.subsystems.drivetrain.Drivetrain.Constants.FREE_SPEED
import com.frcteam3636.bunnybots26.subsystems.drivetrain.Drivetrain.Constants.JOYSTICK_DEADBAND
import com.frcteam3636.bunnybots26.subsystems.drivetrain.Drivetrain.Constants.MODULE_POSITIONS
import com.frcteam3636.bunnybots26.subsystems.drivetrain.Drivetrain.Constants.ROTATION_SENSITIVITY
import com.frcteam3636.bunnybots26.subsystems.drivetrain.Drivetrain.Constants.TRANSLATION_SENSITIVITY
import com.frcteam3636.bunnybots26.subsystems.poseEstimator.Gyro
import com.frcteam3636.bunnybots26.subsystems.poseEstimator.GyroPigeon
import com.frcteam3636.bunnybots26.subsystems.poseEstimator.PhoenixOdometryThread
import com.frcteam3636.bunnybots26.subsystems.poseEstimator.PoseEstimator
import com.frcteam3636.bunnybots26.utils.math.*
import com.frcteam3636.bunnybots26.utils.swerve.*
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import org.team9432.annotation.Logged
import kotlin.jvm.optionals.getOrNull
import kotlin.math.*

@Logged
open class DrivetrainInputs {
    var measuredStatesRelativeToField = PerCorner.generate { SwerveModuleState() }
    var gyroRotation = Rotation2d()
    var gyroVelocity = 0.degreesPerSecond
    var gyroConnected = true
    var measuredStates = PerCorner.generate { SwerveModuleState() }
    var measuredPositions = PerCorner.generate { SwerveModulePosition() }
    var moduleTemperatures = PerCorner.generate {
        SwerveModuleTemperature(0.0.celsius, 0.0.celsius)
    }

    var chassisSpeeds = ChassisSpeeds()
    var chassisSpeedsField = ChassisSpeeds()
    var estimatedPose = Pose2d()
}

interface DrivetrainIO {
    val gyro: Gyro
    val modules: PerCorner<out SwerveModule>
    val kinematics:SwerveDriveKinematics

    val odometryPositions: PerCorner<Array<SwerveModulePosition>>
    val odometryTimestamps: DoubleArray
    val odometryYawPositions: DoubleArray
    val validTimestamps: Int
    val signals: Array<BaseStatusSignal>

    fun getDesiredSpeeds(): ChassisSpeeds
    fun setDesiredSpeeds(speeds: ChassisSpeeds)
    fun getDesiredStates(): PerCorner<SwerveModuleState>
    fun setDesiredStates(states: PerCorner<SwerveModuleState>)

    fun updateInputs(inputs: DrivetrainInputs)
    fun drive(translationInput: Translation2d, rotationInput: Translation2d)
    fun driveWithoutDeadband(translationInput: Translation2d, rotationInput: Translation2d)
    fun stop()
    fun zeroGyro(isReversed: Boolean = false, offset: Rotation2d = Rotation2d.kZero)
    fun runCharacterization(voltage: Voltage, shouldSpin: Boolean = false, shouldStraight: Boolean = false)
}

/** Drivetrain I/O layer that uses real swerve modules along with a Pigeon gyro. */
class DrivetrainIOReal(override val modules: PerCorner<SwerveModule>) : DrivetrainIO {

    val gyroConfig: MountPoseConfigs = MountPoseConfigs()
    override val gyro = GyroPigeon(Pigeon2(CTREDeviceId.PigeonGyro)).apply {
        gyroConfig
    }
    override val kinematics = SwerveDriveKinematics(*MODULE_POSITIONS.map { it.position.translation }.toTypedArray())

    override val odometryPositions get() = modules.map { it.odometryPositions }
    override val odometryTimestamps get() = modules[DrivetrainCorner.FRONT_LEFT].odometryTimestamps
    override val odometryYawPositions: DoubleArray get() = gyro.odometryYawPositions
    override val validTimestamps get() = modules[DrivetrainCorner.FRONT_LEFT].validTimestamps
    override val signals: Array<BaseStatusSignal> get() {
        var signals = arrayOf<BaseStatusSignal>()
        modules.forEach { module -> signals += module.signals }
        signals += gyro.signals
        return signals
    }

    init {
        PhoenixOdometryThread.start()
    }

    override fun getDesiredSpeeds(): ChassisSpeeds = kinematics.cornerStatesToChassisSpeeds(getDesiredStates())

    override fun setDesiredSpeeds(speeds: ChassisSpeeds) {
        val discretized = ChassisSpeeds.discretize(speeds, Robot.period)
        setDesiredStates(kinematics.toCornerSwerveModuleStates(discretized))
    }

    override fun getDesiredStates(): PerCorner<SwerveModuleState> = modules.map { it.desiredState }

    override fun setDesiredStates(states: PerCorner<SwerveModuleState>) {
        val stateArr = states.toTypedArray()
        SwerveDriveKinematics.desaturateWheelSpeeds(stateArr, FREE_SPEED)
        modules.zip(states).forEach { (module, state) -> module.desiredState = state }
    }

    override fun updateInputs(inputs: DrivetrainInputs) {
        gyro.periodic()
        modules.forEachIndexed { i, module ->
            module.periodic()
            inputs.measuredStates[i] = module.state     // no allocation
            inputs.measuredPositions[i] = module.position   // no allocation
            inputs.measuredStatesRelativeToField[i] = SwerveModuleState( module.state.speedMetersPerSecond,
                module.position.angle + gyro.rotation)
        }
        inputs.gyroRotation = gyro.rotation
        inputs.gyroVelocity = gyro.velocity
        inputs.gyroConnected = gyro.connected
        inputs.moduleTemperatures = modules.map { it.temperatures }
        inputs.chassisSpeeds = kinematics.cornerStatesToChassisSpeeds(inputs.measuredStates)
        inputs.chassisSpeedsField = if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) {
            ChassisSpeeds.fromRobotRelativeSpeeds(inputs.chassisSpeeds, inputs.estimatedPose.rotation)
        } else {
            ChassisSpeeds.fromFieldRelativeSpeeds(inputs.chassisSpeeds, inputs.estimatedPose.rotation + Rotation2d.k180deg)
        }
    }

    private fun isInDeadband(translation: Translation2d) =
        abs(translation.x) < JOYSTICK_DEADBAND && abs(translation.y) < JOYSTICK_DEADBAND

    private fun calculateInputCurve(input: Double): Double {
        val exponent = 2.5
        return input.absoluteValue.pow(exponent).withSign(input)
    }

    override fun drive(translationInput: Translation2d, rotationInput: Translation2d) {
        if (isInDeadband(translationInput) && isInDeadband(rotationInput)) {
            // No joystick input - stop moving!
            stop()
        } else {
            driveWithoutDeadband(translationInput, rotationInput)
        }
    }

    override fun driveWithoutDeadband(translationInput: Translation2d, rotationInput: Translation2d) {
        setDesiredSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
            calculateInputCurve(translationInput.x) *
                    FREE_SPEED.inMetersPerSecond() *
                    TRANSLATION_SENSITIVITY,
            calculateInputCurve(translationInput.y) *
                    FREE_SPEED.inMetersPerSecond() *
                    TRANSLATION_SENSITIVITY,
            rotationInput.y * TAU * ROTATION_SENSITIVITY,
            PoseEstimator.inputs.estimatedRotation
        ))
    }

    override fun stop() {
        setDesiredStates(BRAKE_POSITION)
    }

    override fun zeroGyro(isReversed: Boolean, offset: Rotation2d) {
        println("Zeroing gyro")

        // Flip the heading if needed
        var zeroPos =
            when (DriverStation.getAlliance().getOrNull()) {
                DriverStation.Alliance.Red -> Rotation2d.k180deg
                else -> Rotation2d.kZero
            }

        if (isReversed) {
            zeroPos += Rotation2d.k180deg
        }

        PoseEstimator.resetRotation(zeroPos + offset)
    }

    override fun runCharacterization(voltage: Voltage, shouldSpin: Boolean, shouldStraight: Boolean) {
        if (shouldSpin) {
            for ((i, element) in MODULE_POSITIONS.withIndex()) {
                val trans = element.position.translation
                var angle = atan2(trans.y, trans.x)
                if (element == MODULE_POSITIONS.frontRight || element == MODULE_POSITIONS.backRight) {
                    if (element == MODULE_POSITIONS.backRight) {
                        modules[i].characterize(voltage, Rotation2d(angle.radians).unaryMinus().measure)
                    } else {
                        modules[i].characterize(voltage, Rotation2d(angle.radians).unaryMinus().measure + Rotation2d.k180deg.measure)
                    }
                } else {
                    angle += 90.degrees.inRadians()
                    modules[i].characterize(voltage, Rotation2d(angle.radians).measure)
                }
            }
        } else if (shouldStraight) {
            for ((i, element) in MODULE_POSITIONS.withIndex()) {
                modules[i].characterize(voltage, element.position.rotation.unaryMinus().measure)
            }
        } else {
            for (module in modules) {
                module.characterize(voltage, null)
            }
        }
    }

}

/** Drivetrain I/O layer that uses simulated swerve modules along with a simulated gyro with an angle based off their movement. */
class DrivetrainIOSim(
    override val gyro: Gyro,
    override val modules: PerCorner<out SwerveModule>,
    override val kinematics: SwerveDriveKinematics,
    override val odometryPositions: PerCorner<Array<SwerveModulePosition>>,
    override val odometryTimestamps: DoubleArray,
    override val odometryYawPositions: DoubleArray,
    override val validTimestamps: Int,
    override val signals: Array<BaseStatusSignal>
) : DrivetrainIO {
    override fun getDesiredSpeeds(): ChassisSpeeds {
        TODO("Not yet implemented")
    }

    override fun setDesiredSpeeds(speeds: ChassisSpeeds) {
        TODO("Not yet implemented")
    }

    override fun getDesiredStates(): PerCorner<SwerveModuleState> {
        TODO("Not yet implemented")
    }

    override fun setDesiredStates(states: PerCorner<SwerveModuleState>) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: DrivetrainInputs) {
        TODO("Not yet implemented")
    }

    override fun drive(
        translationInput: Translation2d,
        rotationInput: Translation2d
    ) {
        TODO("Not yet implemented")
    }

    override fun driveWithoutDeadband(
        translationInput: Translation2d,
        rotationInput: Translation2d
    ) {
        TODO("Not yet implemented")
    }

    override fun stop() {
        TODO("Not yet implemented")
    }

    override fun zeroGyro(isReversed: Boolean, offset: Rotation2d) {
        TODO("Not yet implemented")
    }

    override fun runCharacterization(
        voltage: Voltage,
        shouldSpin: Boolean,
        shouldStraight: Boolean
    ) {
        TODO("Not yet implemented")
    }

}