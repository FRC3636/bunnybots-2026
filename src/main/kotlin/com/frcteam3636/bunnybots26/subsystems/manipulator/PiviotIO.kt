package com.frcteam3636.bunnybots26.subsystems.manipulator
import com.frcteam3636.bunnybots26.utils.math.amps
import com.frcteam3636.bunnybots26.utils.math.radians
import com.frcteam3636.bunnybots26.utils.math.radiansPerSecond
import org.team9432.annotation.Logged


@Logged
open class PiviotInputs{
    var position = 0.radians
    var velocity = 0.radiansPerSecond
    var current = 0.amps
}

interface PiviotIO{

}