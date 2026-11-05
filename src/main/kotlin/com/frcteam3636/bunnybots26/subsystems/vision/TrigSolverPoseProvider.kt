package com.frcteam3636.bunnybots26.subsystems.vision

import com.frcteam3636.bunnybots26.robot.RobotState
import com.frcteam3636.bunnybots26.subsystems.drivetrain.FIELD_LAYOUT
import com.frcteam3636.bunnybots26.utils.math.inDegreesPerSecond
import com.frcteam3636.bunnybots26.utils.math.inRadians
import com.frcteam3636.bunnybots26.utils.math.radians
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.AngularVelocity
import kotlin.math.*


class TrigSolverPoseProvider(
    name: String,
    private val chassisToCamera: Transform3d,
    private val yawGetter: () -> Rotation2d,
    private val angularVelocityGetter: () -> AngularVelocity,
    @Suppress("unused") private val isLL4: Boolean,
) : AbsolutePoseProvider {

    private val table = NetworkTableInstance.getDefault().getTable(name)
    private val hbSub   = table.getDoubleTopic("hb").subscribe(0.0)
    private val tvSub   = table.getIntegerTopic("tv").subscribe(0L)
    private val txSub   = table.getDoubleTopic("tx").subscribe(0.0)
    private val tidSub  = table.getDoubleTopic("tid").subscribe(0.0)
    private val corners = table.getDoubleArrayTopic("tcornxy").subscribe(doubleArrayOf())

    private var lastHb = 0.0
    private var loopsSinceHb = 0

    override fun updateInputs(inputs: AbsolutePoseProviderInputs) {
        val hb = hbSub.get()
        inputs.connected = hb > lastHb || loopsSinceHb < CONNECTED_TIMEOUT
        if (hb == lastHb) loopsSinceHb++ else loopsSinceHb = 0
        lastHb = hb

        inputs.measurements = arrayOf()
        inputs.observedTags = intArrayOf()

        if (tvSub.get() == 0L) {
            inputs.latestTargetObservation = TargetObservation(targetPresent = false)
            return
        }
        if (RobotState.beforeFirstEnable) return   // yaw not seeded yet

        val tid = tidSub.get().toInt()
        val tagOpt = FIELD_LAYOUT.getTagPose(tid)
        if (tagOpt.isEmpty) return
        val tagPose = tagOpt.get()

        val c = corners.get()
        if (c.size < 8) return // need at least one target's 4 corners

        // Format per Limelight docs: [x_TL, y_TL, x_TR, y_TR, x_BR, y_BR, x_BL, y_BL]

        val yTop    = min(c[1], c[3])
        val yBottom = max(c[5], c[7])
        val ySpanNormalized = yBottom - yTop
        val thetaV = ySpanNormalized / 2.0 * VERTICAL_FOV_RAD
        if (thetaV <= 1e-3) return

        val dOptical = (TAG_HEIGHT_M / 2.0) / tan(thetaV / 2.0)
        val distance = dOptical * DISTANCE_SCALAR

        val latencySeconds = CAMERA_LATENCY_S
        val angularVelRadPerSec =
            angularVelocityGetter().inDegreesPerSecond() * (PI / 180.0)
        val yawAtFrame =
            yawGetter() - Rotation2d(angularVelRadPerSec * latencySeconds)

        val txCorrectedRad = txSub.get().radians * TX_SCALAR
        val camYawRad = chassisToCamera.rotation.toRotation2d().radians
        val angleToTagFieldRad = yawAtFrame.radians + camYawRad + txCorrectedRad.inRadians()

        val tagField = tagPose.toPose2d()
        val camToTagField = Translation2d(distance, Rotation2d(angleToTagFieldRad))
        val fieldToCam = tagField.translation - camToTagField

        val camOffsetRobot = chassisToCamera.translation.toTranslation2d()
        val camOffsetField = camOffsetRobot.rotateBy(yawAtFrame)
        val robotTranslation = fieldToCam - camOffsetField

        val pose = Pose2d(robotTranslation, yawAtFrame)

        val sigmaRadial  = RADIAL_STD_SCALE * distance * distance
        val sigmaLateral = LATERAL_STD_SCALE * distance
        val cosA = cos(angleToTagFieldRad)
        val sinA = sin(angleToTagFieldRad)
        val varX = (sigmaRadial * cosA).pow(2) + (sigmaLateral * sinA).pow(2)
        val varY = (sigmaRadial * sinA).pow(2) + (sigmaLateral * cosA).pow(2)
        val stdDev = VecBuilder.fill(
            sqrt(varX),
            sqrt(varY),
            Double.POSITIVE_INFINITY,
        )

        inputs.measurements = arrayOf(
            AbsolutePoseMeasurement(
                pose = pose,
                timestamp = edu.wpi.first.units.Units.Seconds.of(
                    (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - latencySeconds)
                ),
                stdDeviation = stdDev,
                isLowQuality = false,
            )
        )
        inputs.observedTags = intArrayOf(tid)
        inputs.latestTargetObservation = TargetObservation(
            tx = Rotation2d(txCorrectedRad),
            ty = Rotation2d.kZero,
            targetPresent = true,
        )
    }

    companion object {
        private const val TAG_HEIGHT_M = 0.1651

        private const val VERTICAL_FOV_RAD = 0.854
        private const val CAMERA_LATENCY_S = 0.020

        private const val TX_SCALAR = 38.12 / 35.99

        private const val DISTANCE_SCALAR = 1.0

        private const val RADIAL_STD_SCALE  = 0.15
        private const val LATERAL_STD_SCALE = 0.03

        private const val CONNECTED_TIMEOUT = 250
    }
}