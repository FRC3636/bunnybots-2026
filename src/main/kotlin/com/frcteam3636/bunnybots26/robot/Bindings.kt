package com.frcteam3636.bunnybots26.robot

import com.ctre.phoenix6.SignalLogger
import com.frcteam3636.bunnybots26.subsystems.drivetrain.Drivetrain
import com.frcteam3636.bunnybots26.utils.autos.APTargetWithTolerance
import com.frcteam3636.bunnybots26.utils.math.meters
import com.frcteam3636.bunnybots26.utils.math.radians
import com.revrobotics.util.StatusLogger
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.Preferences
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import kotlin.math.PI

val joystickLeft = CommandJoystick(0)
val joystickRight = CommandJoystick(1)
val controller = CommandXboxController(2)

@Suppress("unused")
val joystickDev = CommandJoystick(3)

@Suppress("unused")
val controllerDev = CommandXboxController(4)

fun configureBindings() {

    /* main bindings */
    joystickRight.button(1).whileTrue(
        Drivetrain.alignWithAutopilot(APTargetWithTolerance(Pose2d(13.34, 4.03, Rotation2d.kZero)))
    )
    /*  default commands   */

//    Drivetrain.defaultCommand = Drivetrain.driveWithJoysticks (
//        joystickLeft.hid,
//        joystickRight.hid
//    )

    Drivetrain.defaultCommand = Drivetrain.driveWithXboxController(controller)

    /* zeroing commands */

    controller.x().onTrue(
        Commands.sequence(
            Commands.runOnce( { println("Zeroing Gyro.") } ),
            Drivetrain.zeroGyro()
        ).ignoringDisable(true)
    )

    joystickLeft.button(8).onTrue(
        Commands.sequence(
            Commands.runOnce( { println("Zeroing Gyro.") } ),
            Drivetrain.zeroGyro()
        ).ignoringDisable(true)
    )

    joystickRight.button(8).onTrue(
        Commands.sequence(
            Commands.runOnce( { println("Pre-match Zeroing.") } ),
            Drivetrain.zeroGyro()
        ).ignoringDisable(true)
    )

    /* dev bindings */

    if (Preferences.getBoolean("DeveloperMode", true)) {
        controllerDev.leftBumper().onTrue(
            Commands.runOnce(SignalLogger::start)
                .andThen(StatusLogger::start)
                .andThen(Commands.runOnce({println("Starting Logging")}))
        )
        controllerDev.rightBumper().onTrue(
            Commands.runOnce(SignalLogger::stop)
            .andThen(StatusLogger::stop)
            .andThen(Commands.runOnce({println("Stopping Logging")})
            )
        )

        controllerDev.y().whileTrue(Drivetrain.sysIdQuasistaticSpin(SysIdRoutine.Direction.kForward))
        controllerDev.a().whileTrue(Drivetrain.sysIdQuasistaticSpin(SysIdRoutine.Direction.kReverse))
        controllerDev.b().whileTrue(Drivetrain.sysIdDynamicSpin(SysIdRoutine.Direction.kForward))
        controllerDev.x().whileTrue(Drivetrain.sysIdDynamicSpin(SysIdRoutine.Direction.kReverse))

        controllerDev.povUp().whileTrue(Drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward))
        controllerDev.povDown().whileTrue(Drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
        controllerDev.povRight().whileTrue(Drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward))
        controllerDev.povLeft().whileTrue(Drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse))

        joystickDev.button(1).whileTrue(Drivetrain.calculateWheelRadius())
    }
}