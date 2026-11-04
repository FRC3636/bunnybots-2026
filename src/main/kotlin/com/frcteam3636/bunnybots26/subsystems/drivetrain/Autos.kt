package com.frcteam3636.bunnybots26.subsystems.drivetrain

import com.frcteam3636.bunnybots26.utils.autos.APTargetWithTolerance
import com.frcteam3636.bunnybots26.utils.autos.flipTarget
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce

interface Auto {
    fun getPath(flipH: Boolean, flipV: Boolean): Command
}


fun resetOdom(pose: APTargetWithTolerance,flipH: Boolean,flipV: Boolean): Command = runOnce({
    Drivetrain.poseEstimator.resetPose(
        flipTarget(pose,flipH = flipH,flipV = flipV).reference
    )
})