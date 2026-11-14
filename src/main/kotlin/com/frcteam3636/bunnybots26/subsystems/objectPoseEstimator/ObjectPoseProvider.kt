package com.frcteam3636.bunnybots26.subsystems.objectPoseEstimator

import org.team9432.annotation.Logged

@Logged
open class ObjectPoseProviderInputs(){
    /* Targets the camera can see*/
    var targets: BasicTargetInfo? = null

    var lastSeenTarget: BasicTargetInfo? = null

    var isConnected: Boolean = false

    var pipeline: PipelineType? = PipelineType.ColorBlob

}


