package com.frcteam3636.bunnybots26.subsystems.objectPoseEstimator

open class PoseProviderTargets<T : BasicTargetInfo>(type: PipelineType){
    var targets: List<T> = emptyList()

    var lastTarget: T? = null

    var isConnected: Boolean = false

    var pipeline: PipelineType = type

}

sealed interface BasicTargetInfo{
    val tv: Double
    val tx: Double
    val ty: Double

}

data class BasicTarget(override val tv: Double, override val tx: Double, override val ty: Double) : BasicTargetInfo{}

data class AprilTagTarget(
    override val tx: Double,
    override val ty: Double,
    override val tv: Double,
    val aprilTagIds: IntArray,


    ) : BasicTargetInfo
enum class PipelineType(){
    AprilTag,
    ObjectDetection,
    ColorBlob,
    Custom
}




//data class targets
//class TargetObservationStruct : Struct<TargetObservation> {
//    override fun getTypeClass(): Class<TargetObservation> = TargetObservation::class.java
//    override fun getTypeName(): String = "struct:TargetObservation"
//    override fun getTypeString(): String = "struct:TargetObservation"
//    override fun getSize(): Int =
//        Rotation2d.struct.size * 2 + kSizeBool
//
//    override fun getSchema(): String = "Rotation2d tx; Rotation2d ty; bool targetPresent;"
//
//    override fun unpack(bb: ByteBuffer): TargetObservation =
//        TargetObservation(
//            Rotation2d.struct.unpack(bb),
//            Rotation2d.struct.unpack(bb),
//            bb.get() != 0.toByte()
//        )
//
//
//    override fun pack(bb: ByteBuffer, value: TargetObservation) {
//        Rotation2d.struct.pack(bb, value.tx)
//        Rotation2d.struct.pack(bb, value.ty)
//        bb.put(if (value.targetPresent) 1 else 0)
//    }
//}