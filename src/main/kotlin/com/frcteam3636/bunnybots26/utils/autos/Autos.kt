package com.frcteam3636.bunnybots26.utils.autos

import com.frcteam3636.bunnybots26.subsystems.drivetrain.Drivetrain
import com.frcteam3636.bunnybots26.utils.math.*
import com.therekrab.autopilot.APTarget
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj.DriverStation
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI

class APTargetWithTolerance(pose: Pose2d) : APTarget(pose) {
    var tolerance: Distance = 25.centimeters

    override fun clone(): APTargetWithTolerance {
        val target = APTargetWithTolerance(m_reference)
        target.m_velocity = m_velocity
        target.m_entryAngle = m_entryAngle
        target.m_rotationRadius = m_rotationRadius
        target.tolerance = tolerance
        return target
    }

    fun withVelocity(velocity: LinearVelocity): APTargetWithTolerance {
        val clone = this.clone()
        clone.m_velocity = velocity.inMetersPerSecond()
        return clone
    }

    override fun withReference(reference: Pose2d): APTargetWithTolerance {
        val target = this.clone()
        target.m_reference = reference
        return target
    }

    fun withTolerance(tolerance: Distance): APTargetWithTolerance {
        val target = this.clone()
        target.tolerance = tolerance
        return target
    }
}

fun flipTargetHorizontal(target: APTarget): APTarget {
    return target.withReference(
        Pose2d(
            Translation2d(
                FIELD_HEIGHT_METERS - target.reference.translation.x, target.reference.translation.y
            ), Rotation2d(PI - target.reference.rotation.radians)
        )
    )
}

fun flipTargetVertical(target: APTarget): APTarget {
    return target.withReference(
        Pose2d(
            Translation2d(
                target.reference.translation.x, FIELD_WIDTH_METERS - target.reference.translation.y.meters.inMeters()
            ), Rotation2d(-target.reference.rotation.radians)
        )
    )
}

fun flipTarget(target: APTarget, flipH: Boolean = false, flipV: Boolean = false): APTarget {
    var new_target = target
    if (flipH) {
        new_target = flipTargetHorizontal(new_target)
    }
    if (flipV) {
        new_target = flipTargetVertical(new_target)
    }
    return new_target
}

fun flipPath(
    path: Array<APTarget>, flipH: Boolean = false, flipV: Boolean = false
): Array<APTarget> {
    if (!(flipH || flipV)) {
        return path
    }
    for (i in 0..<path.size) {
        path[i] = flipTarget(path[i])
    }
    return path
}

fun isRedAlliance() = DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red