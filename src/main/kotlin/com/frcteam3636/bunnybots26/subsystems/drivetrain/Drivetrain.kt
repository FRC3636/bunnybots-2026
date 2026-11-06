package com.frcteam3636.bunnybots26.subsystems.drivetrain

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.SignalLogger
import com.frcteam3636.bunnybots26.CTREDeviceId
import com.frcteam3636.bunnybots26.robot.Robot
import com.frcteam3636.bunnybots26.robot.Robot.Model
import com.frcteam3636.bunnybots26.subsystems.drivetrain.Drivetrain.Constants.BRAKE_POSITION
import com.frcteam3636.bunnybots26.subsystems.drivetrain.Drivetrain.Constants.DRIVE_BASE_RADIUS
import com.frcteam3636.bunnybots26.subsystems.drivetrain.Drivetrain.Constants.MODULE_POSITIONS
import com.frcteam3636.bunnybots26.utils.autos.APTargetWithTolerance
import com.frcteam3636.bunnybots26.utils.autos.flipTargetHorizontal
import com.frcteam3636.bunnybots26.utils.autos.flipTargetVertical
import com.frcteam3636.bunnybots26.utils.fieldRelativeTranslation2d
import com.frcteam3636.bunnybots26.utils.math.*
import com.frcteam3636.bunnybots26.utils.swerve.Corner
import com.frcteam3636.bunnybots26.utils.swerve.PerCorner
import com.frcteam3636.bunnybots26.utils.swerve.translation2dPerSecond
import com.frcteam3636.bunnybots26.utils.translation2d
import com.therekrab.autopilot.APConstraints
import com.therekrab.autopilot.APProfile
import com.therekrab.autopilot.APTarget
import com.therekrab.autopilot.Autopilot
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.Logger
import kotlin.math.abs
import kotlin.math.hypot

enum class FieldSide {
    Left,
    Right
}

object Drivetrain : Subsystem {
    private val io =
        when (Robot.model) {
            Model.SIMULATION -> DrivetrainIOSim()
            Model.COMPETITION ->
                DrivetrainIOReal(
                    MODULE_POSITIONS.zip(Constants.MODULE_CAN_IDS).map { (corner, ids) ->
                        val (driveId, turnId, encoderId) = ids
                        Mk5nSwerveModule(
                            DrivingTalon(driveId),
                            TurningTalon(turnId, encoderId, corner.magnetOffset),
                            corner.position.rotation
                        )
                    }
                )
        }

    val inputs = LoggedDrivetrainInputs()

    val signals: Array<BaseStatusSignal>
        get() = io.signals

    override fun periodic() {
        io.updateInputs(inputs)

        Logger.processInputs("Drivetrain/Inputs", inputs)
        Logger.recordOutput("Drivetrain/DesiredSpeeds", io.getDesiredSpeeds())
        Logger.recordOutput("Drivetrain/DesiredStates", *io.getDesiredStates().toTypedArray())
    }

    @Suppress("unused")
    fun driveWithJoysticks(translationJoystick: Joystick, rotationJoystick: Joystick): Command =
        run {
            // Directly accessing Joystick.x/y gives inverted values - use a Translation2d instead.
            io.drive(
                translationJoystick.fieldRelativeTranslation2d,
                rotationJoystick.translation2d
            )
        }

    @Suppress("unused")
    fun driveWithController(controller: CommandXboxController): Command = run {
        val translationInput = Translation2d(controller.leftX, controller.leftY)
        val rotationInput = Translation2d(controller.rightY, controller.rightX)

        io.drive(translationInput, rotationInput)
    }

    fun zeroGyro(isReversed: Boolean = false, offset: Rotation2d = Rotation2d.kZero): Command = Commands.runOnce(
        { io.zeroGyro(isReversed, offset) }
    )

    fun stop(): Command = Commands.run(
        { io.setDesiredStates(BRAKE_POSITION) }
    )

    fun resetPose(pose: Pose2d): Command = Commands.runOnce(
        { io.resetEstimatedPose(pose) }
    )

    // --- AUTOPILOT ---

    val autoPilotConstraints: APConstraints =
        APConstraints().withVelocity(7.0).withAcceleration(10.0).withJerk(5.0)

    val autoPilotProfile: APProfile =
        APProfile(autoPilotConstraints)
            .withErrorXY(30.centimeters)
            .withErrorTheta(10.degrees)
            .withBeelineRadius(20.centimeters)

    var autoPilot = Autopilot(autoPilotProfile)

    val autoPilotRotationPID = PIDController(PIDGains(2.0, 0.0, 0.05)).apply { enableContinuousInput(0.0, TAU) }

    fun alignWithAutopilot(target: APTargetWithTolerance): Command {
        return run {
            val velocityVector = inputs.chassisSpeedsField.translation2dPerSecond.toVector()
            val vectorToTarget =
                (inputs.estimatedPose.translation - target.reference.translation).toVector()
            // If we are moving away from the target, stop the robot immediately
            val adjustedChassisSpeeds = inputs.chassisSpeeds.apply { // do not know if this mod
                if (vectorToTarget.dot(velocityVector) <= 0) {
                    vxMetersPerSecond = 0.0
                    vyMetersPerSecond = 0.0
                }
            }

            autoPilot = Autopilot(autoPilotProfile.withErrorXY(target.tolerance))

            val output = autoPilot.calculate(inputs.estimatedPose, inputs.chassisSpeeds, target)

            val rotationOutput =
                autoPilotRotationPID.calculate(
                    inputs.estimatedRotation.radians,
                    output.targetAngle.radians
                )

            io.setDesiredSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                output.vx,
                output.vy,
                rotationOutput.radiansPerSecond,
                inputs.estimatedRotation
            ))

            Logger.recordOutput("Drivetrain/Auto/Output", output)
            Logger.recordOutput("Drivetrain/Auto/RotationOutput", rotationOutput)
            Logger.recordOutput("Drivetrain/Auto/Target", target.reference)
            Logger.recordOutput("Drivetrain/Auto/TargetEndVelocity", target.velocity)
        }
            .until { autoPilot.atTarget(inputs.estimatedPose, target) }
            .finallyDo { -> io.setDesiredStates(BRAKE_POSITION) }
    }

    fun alignAndFlip(target: APTarget, flipH: Boolean, flipV: Boolean): Command {
        var transformedTarget = target
        if (flipH) transformedTarget = flipTargetHorizontal(transformedTarget)
        if (flipV) transformedTarget = flipTargetVertical(transformedTarget)
        return alignWithAutopilot(transformedTarget as APTargetWithTolerance)
    }

    private val limiter = SlewRateLimiter(0.05)
    private var wheelRadiusModuleStates = DoubleArray(4)
    private var wheelRadiusLastAngle = Rotation2d.kZero
    private var wheelRadiusGyroDelta = 0.0

    @Suppress("unused")
    fun calculateWheelRadius(): Command =
        Commands.parallel(
            Commands.sequence(
                Commands.runOnce({ limiter.reset(0.0) }),
                run {
                    val speed = limiter.calculate(0.25)
                    io.driveWithoutDeadband(Translation2d.kZero, Translation2d(0.0, speed))
                }
            ),
            Commands.sequence(
                // Wait for modules to orient
                Commands.waitSeconds(1.0),
                Commands.runOnce({
                    Logger.recordOutput(
                        "Drivetrain/WheelCalculator/Running",
                        true
                    )
                    val modules = io.modules.toTypedArray()
                    for (i in 0..3) {
                        wheelRadiusModuleStates[i] =
                            modules[i].angularDrivePosition.inRadians()
                    }
                    wheelRadiusLastAngle = inputs.gyroRotation
                    wheelRadiusGyroDelta = 0.0
                }),
                Commands.run({
                    val rotation = inputs.gyroRotation
                    wheelRadiusGyroDelta +=
                        abs(rotation.minus(wheelRadiusLastAngle).radians)
                    wheelRadiusLastAngle = rotation
                    Logger.recordOutput(
                        "Drivetrain/WheelCalculator/Gyro Delta",
                        wheelRadiusGyroDelta
                    )
                })
                    .finallyDo { ->
                        var wheelDelta = 0.0
                        // Someone give me a better way to do this
                        for (i in 0..3) {
                            wheelDelta +=
                                abs(
                                    io.modules.toTypedArray()[i]
                                        .angularDrivePosition
                                        .inRadians() -
                                            wheelRadiusModuleStates[i]
                                ) / 4.0
                        }

                        Logger.recordOutput(
                            "Drivetrain/WheelCalculator/InitialPosition",
                            wheelRadiusModuleStates
                        )
                        Logger.recordOutput(
                            "Drivetrain/WheelCalculator/FinalPosition",
                            io.modules
                                .map { it.angularDrivePosition.inRadians() }
                                .toDoubleArray() // AKit doesn't accept
                            // united arrays?
                        )

                        val wheelRadius =
                            ((wheelRadiusGyroDelta * DRIVE_BASE_RADIUS) /
                                    wheelDelta)
                        Logger.recordOutput(
                            "Drivetrain/WheelCalculator/WheelDelta",
                            wheelDelta
                        )
                        Logger.recordOutput(
                            "Drivetrain/WheelCalculator/CalibratedRadius",
                            wheelRadius.meters
                        )
                        Logger.recordOutput(
                            "Drivetrain/WheelCalculator/Running",
                            false
                        )
                    }
            )
        )

    var sysID =
        SysIdRoutine(
            SysIdRoutine.Config(null, null, null) {
                SignalLogger.writeString("state", it.toString())
            },
            SysIdRoutine.Mechanism(
                io::runCharacterization,
                null,
                this,
            )
        )

    @Suppress("unused")
    fun sysIdQuasistatic(direction: SysIdRoutine.Direction) =
        run { io.runCharacterization(0.volts, shouldStraight = true) }
            .withTimeout(2.0)
            .andThen(sysID.quasistatic(direction))!!

    @Suppress("unused")
    fun sysIdDynamic(direction: SysIdRoutine.Direction) =
        run { io.runCharacterization(0.volts, shouldStraight = true) }
            .withTimeout(2.0)
            .andThen(sysID.dynamic(direction))!!

    @Suppress("unused")
    fun sysIdQuasistaticSpin(direction: SysIdRoutine.Direction) =
        run { io.runCharacterization(0.volts, shouldSpin = true) }
            .withTimeout(2.0)
            .andThen(sysID.quasistatic(direction))!!

    @Suppress("unused")
    fun sysIdDynamicSpin(direction: SysIdRoutine.Direction) =
        run { io.runCharacterization(0.volts, shouldSpin = true) }
            .withTimeout(2.0)
            .andThen(sysID.dynamic(direction))!!

    @Suppress("unused")
    object Constants {
        /** Unit: Percent of max robot speed */
        const val TRANSLATION_SENSITIVITY = 1.0

        /** Unit: Rotations per second */
        const val ROTATION_SENSITIVITY = 1.0

        const val JOYSTICK_DEADBAND = 0.3

        val FRONT_LEFT_CONSTANTS = TunerConstants.FrontLeft!!
        val FRONT_RIGHT_CONSTANTS = TunerConstants.FrontRight!!
        val BACK_RIGHT_CONSTANTS = TunerConstants.BackRight!!
        val BACK_LEFT_CONSTANTS = TunerConstants.BackLeft!!

        val FRONT_LEFT_MAGNET_OFFSET = FRONT_LEFT_CONSTANTS.EncoderOffset
        val FRONT_RIGHT_MAGNET_OFFSET = FRONT_RIGHT_CONSTANTS.EncoderOffset
        val BACK_RIGHT_MAGNET_OFFSET = BACK_RIGHT_CONSTANTS.EncoderOffset
        val BACK_LEFT_MAGNET_OFFSET = BACK_LEFT_CONSTANTS.EncoderOffset

        val MODULE_POSITIONS = PerCorner(
            frontLeft =
                Corner(Pose2d(Translation2d(FRONT_LEFT_CONSTANTS.LocationX, FRONT_LEFT_CONSTANTS.LocationY), Rotation2d.kZero), FRONT_LEFT_MAGNET_OFFSET),
            frontRight =
                Corner(Pose2d(Translation2d(FRONT_RIGHT_CONSTANTS.LocationX, FRONT_RIGHT_CONSTANTS.LocationY), Rotation2d.fromDegrees(180.0)), FRONT_RIGHT_MAGNET_OFFSET),
            backLeft =
                Corner(Pose2d(Translation2d(BACK_LEFT_CONSTANTS.LocationX, BACK_LEFT_CONSTANTS.LocationY), Rotation2d.kZero), BACK_LEFT_MAGNET_OFFSET),
            backRight =
                Corner(Pose2d(Translation2d(BACK_RIGHT_CONSTANTS.LocationX, BACK_RIGHT_CONSTANTS.LocationY), Rotation2d.fromDegrees(180.0)), BACK_RIGHT_MAGNET_OFFSET),
        )

        val MODULE_CAN_IDS = PerCorner(
            frontLeft =
                Triple(CTREDeviceId.FrontLeftDrivingMotor, CTREDeviceId.FrontLeftTurningMotor, CTREDeviceId.FrontLeftTurningEncoder),
            frontRight =
                Triple(CTREDeviceId.FrontRightDrivingMotor, CTREDeviceId.FrontRightTurningMotor, CTREDeviceId.FrontRightTurningEncoder),
            backLeft =
                Triple(CTREDeviceId.BackLeftDrivingMotor, CTREDeviceId.BackLeftTurningMotor, CTREDeviceId.BackLeftTurningEncoder),
            backRight =
                Triple(CTREDeviceId.BackRightDrivingMotor, CTREDeviceId.BackRightTurningMotor, CTREDeviceId.BackRightTurningEncoder),
        )

        val ROBOT_LENGTH = 25.5.inches
        val ROBOT_WIDTH = 25.5.inches
        val BUMPER_WIDTH = 30.inches
        val BUMPER_LENGTH = 30.inches
        val DRIVE_BASE_RADIUS =
            hypot(MODULE_POSITIONS.frontLeft.position.x, MODULE_POSITIONS.frontLeft.position.y)

        const val WHEEL_COF = 2.255

        val FREE_SPEED = TunerConstants.kSpeedAt12Volts

        val PATH_FOLLOWING_TRANSLATION_GAINS = PIDGains(8.0, 0.0, 0.0)
        val PATH_FOLLOWING_ROTATION_GAINS = PIDGains(10.0, 0.0, 0.0)

        val BRAKE_POSITION =
            MODULE_POSITIONS.map { module -> SwerveModuleState(0.0, module.position.translation.angle) }

        val ALIGN_TARGET = APTargetWithTolerance(
            Pose2d(Translation2d((546.87 + 48.0).inches, 158.3.inches), Rotation2d.k180deg)
        )
    }
}