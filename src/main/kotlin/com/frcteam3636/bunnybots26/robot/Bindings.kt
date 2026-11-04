package com.frcteam3636.bunnybots26.robot

import com.ctre.phoenix6.SignalLogger
import com.frcteam3636.bunnybots26.subsystems.drivetrain.Drivetrain
import com.frcteam3636.bunnybots26.subsystems.intake.Intake
import com.revrobotics.util.StatusLogger
import edu.wpi.first.wpilibj.Preferences
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine

val joystickLeft = CommandJoystick(0)
val joystickRight = CommandJoystick(1)
val controller = CommandXboxController(2)

@Suppress("unused")
val joystickDev = CommandJoystick(3)

@Suppress("unused")
val controllerDev = CommandXboxController(4)

fun configureBindings() {

    /* main bindings */


    /*  default commands   */

    Drivetrain.defaultCommand = Drivetrain.driveWithJoysticks (
        joystickLeft.hid,
        joystickRight.hid
    )

    /* zeroing commands */

    joystickLeft.button(8).onTrue(Commands.runOnce({
        println("Gyro Zeroed")
        Drivetrain.zeroGyro()
    }).ignoringDisable(true))

    joystickRight.button(8).onTrue(
        Commands.sequence(
            Commands.runOnce({
                println("Pre-match Zeroing.")
                Drivetrain.zeroGyro()
            }).ignoringDisable(true),

        )
    )

    /* dev bindings */

    if (Preferences.getBoolean("DeveloperMode", false)) {
        controllerDev.leftBumper().onTrue(
            Commands.runOnce(SignalLogger::start)
                .andThen(StatusLogger::start)
        )
        controllerDev.rightBumper().onTrue(
            Commands.runOnce(SignalLogger::stop)
                .andThen(StatusLogger::stop)
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