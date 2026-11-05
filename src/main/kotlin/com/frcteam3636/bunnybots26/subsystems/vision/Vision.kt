package com.frcteam3636.bunnybots26.subsystems.vision

import com.frcteam3636.bunnybots26.robot.Robot
import com.frcteam3636.bunnybots26.robot.Robot.Model
import com.frcteam3636.bunnybots26.robot.RobotState
import com.frcteam3636.bunnybots26.subsystems.drivetrain.Drivetrain
import com.frcteam3636.bunnybots26.subsystems.drivetrain.FIELD_LAYOUT
import com.frcteam3636.bunnybots26.utils.math.degrees
import com.frcteam3636.bunnybots26.utils.math.meters
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

object Vision : Subsystem {

    val inputs = LoggedVisionInputs()

    private lateinit var poseSupplier: () -> Pose2d
    private lateinit var enabledSupplier: () -> Boolean

    private val providers =
        when (Robot.model) {
            Model.SIMULATION ->
                mapOf(
                    "SimCamera" to CameraSimPoseProvider("limelight", Transform3d.kZero)
                )
            else ->
                mapOf(
                    "Left" to
                            LimelightPoseProvider(
                                "limelight-left",
                                { poseSupplier().rotation },
                                { Drivetrain.inputs.gyroVelocity },
                                { Drivetrain.inputs.gyroConnected },
                                isLL4 = true,
                            ),
                    "Right" to
                            LimelightPoseProvider(
                                "limelight-right",
                                { poseSupplier().rotation },
                                { Drivetrain.inputs.gyroVelocity },
                                { Drivetrain.inputs.gyroConnected },
                                isLL4 = true,
                            ),
                    "LeftTrig" to
                            TrigSolverPoseProvider(
                                "limelight-left",
                                chassisToCamera = LEFT_CAMERA_TRANSFORM,
                                yawGetter = { poseSupplier().rotation },
                                angularVelocityGetter = { Drivetrain.inputs.gyroVelocity },
                                isLL4 = true,
                            ),
                    "RightTrig" to
                            TrigSolverPoseProvider(
                                "limelight-left",
                                chassisToCamera = LEFT_CAMERA_TRANSFORM,
                                yawGetter = { poseSupplier().rotation },
                                angularVelocityGetter = { Drivetrain.inputs.gyroVelocity },
                                isLL4 = true,
                            ),
                )
        }.mapValues { Pair(it.value, LoggedAbsolutePoseProviderInputs()) }

    /** Must be called once from Drivetrain.init (or RobotContainer). */
    fun registerPoseSupplier(
        supplier: () -> Pose2d,
        enabledSupplier: () -> Boolean,
    ) {
        this.poseSupplier = supplier
        this.enabledSupplier = enabledSupplier
    }

    val allProvidersConnected: Boolean
        get() = providers.values.all { it.second.connected }

    private val accepted = mutableListOf<AbsolutePoseMeasurement>()
    private val rejected = mutableListOf<AbsolutePoseMeasurement>()
    private val acceptedPoses = mutableListOf<Pose2d>()
    private val rejectedPoses = mutableListOf<Pose2d>()
    private val observedTags = mutableSetOf<Int>()

    override fun periodic() {
        if (!::poseSupplier.isInitialized) return

        accepted.clear(); rejected.clear()
        acceptedPoses.clear(); rejectedPoses.clear()
        observedTags.clear()

        val currentPose = poseSupplier()
        val beforeFirstEnable = RobotState.beforeFirstEnable

        for ((name, pair) in providers) {
            val (io, ins) = pair
            io.updateInputs(ins)
            Logger.processInputs("Vision/$name", ins)

            observedTags.addAll(ins.observedTags.toList())

            for (m in ins.measurements) {
                when {
                    m.isLowQuality -> reject(m)
                    m.pose.x < 0.0 || m.pose.y < 0.0 -> reject(m)
                    m.pose.x > FIELD_LAYOUT.fieldLength || m.pose.y > FIELD_LAYOUT.fieldWidth -> reject(m)
                    !beforeFirstEnable && abs(m.pose.rotation.degrees - currentPose.rotation.degrees) > 5.0 -> reject(m)
                    else -> accept(m)
                }
            }

            Logger.recordOutput(
                "Vision/$name/Accepted", *acceptedPoses.toTypedArray()
            )
            Logger.recordOutput(
                "Vision/$name/Rejected", *rejectedPoses.toTypedArray()
            )
            acceptedPoses.clear(); rejectedPoses.clear()
        }

        inputs.acceptedMeasurements = accepted.toTypedArray()
        inputs.rejectedMeasurements = rejected.toTypedArray()
        inputs.observedTags = observedTags.toIntArray()

        Logger.recordOutput(
            "Vision/ObservedTags",
            *FIELD_LAYOUT.tags
                .filter { it.ID in observedTags }
                .map { it.pose }
                .toTypedArray()
        )
    }

    private fun accept(m: AbsolutePoseMeasurement) {
        accepted.add(m)
        acceptedPoses.add(m.pose)
    }

    private fun reject(m: AbsolutePoseMeasurement) {
        rejected.add(m)
        rejectedPoses.add(m.pose)
    }

    /** Consumed by Drivetrain once per loop. */
    fun drainAcceptedMeasurements(): List<AbsolutePoseMeasurement> {
        val out = inputs.acceptedMeasurements.toList()
        inputs.acceptedMeasurements = arrayOf()
        return out
    }

    private val LEFT_CAMERA_TRANSFORM = Transform3d(Translation3d(0.meters,0.meters,0.meters), Rotation3d(Rotation2d(0.degrees)))

    private val RIGHT_CAMERA_TRANSFORM = Transform3d(Translation3d(0.meters,0.meters,0.meters), Rotation3d(Rotation2d(0.degrees)))
}