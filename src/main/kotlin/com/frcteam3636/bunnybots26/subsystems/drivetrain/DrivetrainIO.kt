package com.frcteam3636.bunnybots26.subsystems.drivetrain

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.MountPoseConfigs
import com.frcteam3636.bunnybots26.CTREDeviceId
import com.frcteam3636.bunnybots26.Diagnostics
import com.frcteam3636.bunnybots26.Pigeon2
import com.frcteam3636.bunnybots26.robot.Robot
import com.frcteam3636.bunnybots26.robot.Robot.odometryLock
import com.frcteam3636.bunnybots26.subsystems.drivetrain.Drivetrain.Constants.BRAKE_POSITION
import com.frcteam3636.bunnybots26.subsystems.drivetrain.Drivetrain.Constants.FREE_SPEED
import com.frcteam3636.bunnybots26.subsystems.drivetrain.Drivetrain.Constants.JOYSTICK_DEADBAND
import com.frcteam3636.bunnybots26.subsystems.drivetrain.Drivetrain.Constants.MODULE_POSITIONS
import com.frcteam3636.bunnybots26.subsystems.drivetrain.Drivetrain.Constants.ROTATION_SENSITIVITY
import com.frcteam3636.bunnybots26.subsystems.drivetrain.Drivetrain.Constants.TRANSLATION_SENSITIVITY
import com.frcteam3636.bunnybots26.subsystems.drivetrain.FieldSide
import com.frcteam3636.bunnybots26.subsystems.vision.PhoenixOdometryThread
import com.frcteam3636.bunnybots26.subsystems.vision.Vision
import com.frcteam3636.bunnybots26.subsystems.vision.Vision.FIELD_LAYOUT
import com.frcteam3636.bunnybots26.subsystems.vision.addAbsolutePoseMeasurement
import com.frcteam3636.bunnybots26.utils.autos.FIELD_WIDTH_METERS
import com.frcteam3636.bunnybots26.utils.math.*
import com.frcteam3636.bunnybots26.utils.swerve.DrivetrainCorner
import com.frcteam3636.bunnybots26.utils.swerve.PerCorner
import com.frcteam3636.bunnybots26.utils.swerve.SwerveModuleTemperature
import com.frcteam3636.bunnybots26.utils.swerve.cornerStatesToChassisSpeeds
import com.frcteam3636.bunnybots26.utils.swerve.toCornerSwerveModuleStates
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.COTS
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig
import org.littletonrobotics.junction.Logger
import org.photonvision.simulation.VisionSystemSim
import org.team9432.annotation.Logged
import kotlin.compareTo
import kotlin.jvm.optionals.getOrNull
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.withSign

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

    // new stuff
    var chassisSpeeds = ChassisSpeeds()
    var chassisSpeedsField = ChassisSpeeds()
    var estimatedPose = Pose2d()
    /// NOT necessarily equivilant to estimatedPose.rotation.
    var estimatedRotation = Rotation2d()
}

fun isInDeadband(translation: Translation2d) =
    abs(translation.x) < JOYSTICK_DEADBAND && abs(translation.y) < JOYSTICK_DEADBAND

fun calculateInputCurve(input: Double): Double {
    val exponent = 2.5

    return input.absoluteValue.pow(exponent).withSign(input)
}

abstract class DrivetrainIO {
    protected abstract val gyro: Gyro
    abstract val modules: PerCorner<out SwerveModule>

    abstract fun odometry(inputs: DrivetrainInputs)

    init {
        PhoenixOdometryThread.start()
    }

    // Helper for converting a desired drivetrain velocity into the speeds and angles for each swerve module
    val kinematics =
        SwerveDriveKinematics(*MODULE_POSITIONS.map { it.position.translation }.toTypedArray())

    // Helper for estimating the location of the drivetrain on the field, combining odometry and vision
    val poseEstimator =
        SwerveDrivePoseEstimator(
            kinematics, // swerve drive kinematics
            gyro.rotation, // initial gyro rotation
            PerCorner.generate { SwerveModulePosition() }.toTypedArray(), // initial module positions
            Pose2d.kZero, // initial pose
            VecBuilder.fill(0.02, 0.02, 0.005),
            // Overwrite each measurement
            VecBuilder.fill(0.0, 0.0, 0.0)
        )

    fun getDesiredSpeeds(): ChassisSpeeds = kinematics.cornerStatesToChassisSpeeds(getDesiredStates())
    fun setDesiredSpeeds(speeds: ChassisSpeeds) {
        val discretized = ChassisSpeeds.discretize(speeds, Robot.period)
        setDesiredStates(kinematics.toCornerSwerveModuleStates(discretized))
    }

    fun getDesiredStates(): PerCorner<SwerveModuleState> = modules.map { it.desiredState }
    fun setDesiredStates(states: PerCorner<SwerveModuleState>) {
        val stateArr = states.toTypedArray()
        SwerveDriveKinematics.desaturateWheelSpeeds(stateArr, FREE_SPEED)
        modules.zip(states).forEach { (module, state) -> module.desiredState = state }
    }

    val odometryPositions: PerCorner<Array<SwerveModulePosition>>
        get() = modules.map { it.odometryPositions }

    val odometryTimestamps: DoubleArray
        get() = modules[DrivetrainCorner.FRONT_LEFT].odometryTimestamps

    val odometryYawPositions: DoubleArray
        get() = gyro.odometryYawPositions

    val validTimestamps: Int
        get() = modules[DrivetrainCorner.FRONT_LEFT].validTimestamps

    val signals: Array<BaseStatusSignal>
        get() {
            var signals = arrayOf<BaseStatusSignal>()

            modules.forEach { module ->
                signals += module.signals
            }

            signals += gyro.signals
            return signals
        }

    val lastModulePositions = Array(4) { SwerveModulePosition() }
    val modulePositions = Array(4) { SwerveModulePosition() }
    val moduleDeltas = Array(4) { SwerveModulePosition() }

    private var estimatedRotation = Rotation2d()

    fun fieldSide(): FieldSide {
        val rightSide =
            (poseEstimator.estimatedPosition.y > FIELD_WIDTH_METERS / 2.0).xor(
                DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red
            )
        return if (rightSide) {
            FieldSide.Right
        } else {
            FieldSide.Left
        }
    }

    // sim drive can get lost
    fun drive(translationInput: Translation2d, rotationInput: Translation2d) {
        if (isInDeadband(translationInput) && isInDeadband(rotationInput)) {
            // No joystick input - stop moving!
            stop()
        } else {
            driveWithoutDeadband(translationInput, rotationInput)
        }
    }

    @Suppress("SameParameterValue")
    fun driveWithoutDeadband(
        translationInput: Translation2d,
        rotationInput: Translation2d
    ) {
        setDesiredSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
            calculateInputCurve(translationInput.x) *
                    FREE_SPEED.inMetersPerSecond() *
                    TRANSLATION_SENSITIVITY,
            calculateInputCurve(translationInput.y) *
                    FREE_SPEED.inMetersPerSecond() *
                    TRANSLATION_SENSITIVITY,
            rotationInput.y * TAU * ROTATION_SENSITIVITY,
            estimatedRotation
        ))
    }

    fun updateInputs(inputs: DrivetrainInputs) {
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

        // new stuff
        inputs.estimatedPose = poseEstimator.estimatedPosition
        inputs.chassisSpeeds = kinematics.cornerStatesToChassisSpeeds(inputs.measuredStates)
        inputs.chassisSpeedsField = if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Blue) {
            ChassisSpeeds.fromRobotRelativeSpeeds(inputs.chassisSpeeds, inputs.estimatedPose.rotation)
        } else {
            ChassisSpeeds.fromFieldRelativeSpeeds(inputs.chassisSpeeds, inputs.estimatedPose.rotation + Rotation2d.k180deg)
        }
        inputs.estimatedRotation = inputs.estimatedPose.rotation; // could get overriden by odom function

        estimatedRotation = inputs.estimatedRotation

        odometry(inputs)

        for (m in Vision.drainAcceptedMeasurements()) {
            poseEstimator.addAbsolutePoseMeasurement(m)
        }
    }

    fun zeroGyro(isReversed: Boolean = false, offset: Rotation2d = Rotation2d.kZero) {
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

        poseEstimator.resetRotation(zeroPos + offset)
    }

    fun resetEstimatedPose(pose: Pose2d) {
        poseEstimator.resetPose(pose)
    }

    @Suppress("unused")
    fun stop() {
        setDesiredStates(BRAKE_POSITION)
    }

    fun runCharacterization(voltage: Voltage, shouldSpin: Boolean = false, shouldStraight: Boolean = false) {
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

/** Drivetrain I/O layer that uses real swerve modules along with a Pigeon gyro. */
class DrivetrainIOReal(override val modules: PerCorner<SwerveModule>) : DrivetrainIO() {
    val gyroConfig: MountPoseConfigs = MountPoseConfigs()
    override val gyro = GyroPigeon(Pigeon2(CTREDeviceId.PigeonGyro)).apply {
        gyroConfig
    }

    override fun odometry(inputs: DrivetrainInputs) {
        try {
            odometryLock.lock()
            Logger.recordOutput("Drivetrain/SpeedsFieldRelative", inputs.chassisSpeedsField)
            Logger.recordOutput("Drivetrain/ValidTimestamps", validTimestamps)
            for (i in 0 ..< validTimestamps) {
                for (index in 0..3) {
                    val pos = odometryPositions[index][i]
                    modulePositions[index].distanceMeters = pos.distanceMeters
                    modulePositions[index].angle = pos.angle
                }

                // Compute deltas in-place
                for (index in 0..3) {
                    val deltaDistance =
                        modulePositions[index].distanceMeters -
                                lastModulePositions[index].distanceMeters
                    val deltaAngle =
                        modulePositions[index].angle - lastModulePositions[index].angle
                    moduleDeltas[index].distanceMeters = deltaDistance
                    moduleDeltas[index].angle = modulePositions[index].angle

                    // Update last positions
                    lastModulePositions[index] =
                        SwerveModulePosition(
                            modulePositions[index].distanceMeters,
                            modulePositions[index].angle
                        )
                }

                inputs.estimatedRotation =
                    if (inputs.gyroConnected) {
                        Rotation2d(odometryYawPositions[i].degrees)
                    } else {
                        inputs.estimatedRotation.plus(
                            Rotation2d(
                                kinematics.toTwist2d(*moduleDeltas).dtheta.radians
                            )
                        )
                    }
                poseEstimator.updateWithTime(
                    odometryTimestamps[i],
                    inputs.estimatedRotation,
                    modulePositions
                )
            }
        } finally {
            odometryLock.unlock()
        }

    }
}

/** Drivetrain I/O layer that uses simulated swerve modules along with a simulated gyro with an angle based off their movement. */
class DrivetrainIOSim : DrivetrainIO() {
    val vision = VisionSystemSim("main").apply {
        addAprilTags(FIELD_LAYOUT)
    }

    val driveTrainSimulationConfig: DriveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
        .withGyro(COTS.ofPigeon2())
        .withSwerveModule(COTS.ofMark4n(
            DCMotor.getKrakenX60(1),
            DCMotor.getKrakenX44(1),
            COTS.WHEELS.SLS_PRINTED_WHEELS.cof,
            3
        ))
        .withTrackLengthTrackWidth(
            Drivetrain.Constants.ROBOT_LENGTH,
            Drivetrain.Constants.ROBOT_WIDTH
        )
        .withBumperSize(
            Drivetrain.Constants.BUMPER_LENGTH,
            Drivetrain.Constants.BUMPER_WIDTH
        ).withRobotMass(
            88.581.pounds
        )

    val swerveDriveSimulation: SwerveDriveSimulation = SwerveDriveSimulation(
        driveTrainSimulationConfig,
        Pose2d(3.0, 3.0, Rotation2d()
        )
    )


    val simulatedDrive: SelfControlledSwerveDriveSimulation = SelfControlledSwerveDriveSimulation(swerveDriveSimulation)

    fun updateChassisSpeeds(chassisSpeeds: ChassisSpeeds) {
        simulatedDrive.runChassisSpeeds(chassisSpeeds, Translation2d(), true, false)
    }

    override val modules = PerCorner.generate { SimSwerveModule() }
    override val gyro = GyroMapleSim(swerveDriveSimulation.gyroSimulation)
    // this shit did not work anyway ):<
//    override fun updateInputs(inputs: DrivetrainInputs) {
//        super.updateInputs(inputs)
//        vision.update(poseEstimator.estimatedPosition)
//        Logger.recordOutput("FieldSimulation/RobotPosition", swerveDriveSimulation.simulatedDriveTrainPose)
//        Diagnostics.report(gyro)
//    }

    override fun odometry(inputs: DrivetrainInputs) {
        // idk
    }

    init {
        SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.driveTrainSimulation)
    }
}