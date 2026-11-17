package com.frcteam3636.bunnybots26.subsystems.objectPoseEstimator

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.AngularVelocity
import org.team9432.annotation.Logged

@Logged
open class ObjectPoseProviderInputs(){
    /* Targets the camera can see*/
    var targets: Array<TargetObject> = arrayOf()

    var lastSeenTarget: TargetObject? = null

    var isConnected: Boolean = false

    var objectSeen: Array<TargetType> = arrayOf()

    var pipelineType: PipelineType = PipelineType.ColorBlob

    var targetType: TargetType = TargetType.BlueCone

}

interface ObjectPoseProvider{
    fun updateInputs(inputs: ObjectPoseProviderInputs)
}

class LimeLightObjectPoseProvider(
        name: String,
        private val yawGetter: () -> Rotation2d,
        private val velocityGetter: () -> AngularVelocity,
        private val gyroConnectionGetter: () -> Boolean,
        private val isLL4: Boolean
) : ObjectPoseProvider{
    private var observedTargets: Array<TargetObject> = arrayOf()

    private var lastSeenHeartBeat: Double = 0.0
    private val table = NetworkTableInstance.getDefault().getTable(name)
    private val rawDetections = table.getDoubleArrayTopic("rawdetections").subscribe(doubleArrayOf())

    private val gyroPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish()
    private val throttlePublisher = table.getIntegerTopic("throttle_set").publish()
    private val imuModePublisher = table.getIntegerTopic("imumode_set").publish()
    private val imuAlphaPublisher = table.getDoubleTopic("imuassistalpha_set").publish()



    override fun updateInputs(inputs: ObjectPoseProviderInputs) {
        TODO("Not yet implemented")
    }


}


data class TargetObject(
    val tx: Double,
    val ty: Double,
    val txnc: Double,
    val tync: Double,
    val ta: Double,
)

enum class TargetType(val value: Int){
    RedCone(1),
    BlueCone(2)
}

enum class PipelineType(name: String){
    ColorBlob("color_blob")
}