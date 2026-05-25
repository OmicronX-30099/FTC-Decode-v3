@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Util

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.ftc.components.LoopTimeComponent
import kotlin.math.atan2
import kotlin.math.hypot

fun NextFTCOpMode.addSubsystems(vararg subsystems: Subsystem) =
    addComponents(BindingsComponent, BulkReadComponent, LoopTimeComponent(), SubsystemComponent(*subsystems))
fun NextFTCOpMode.includePedro(followerFactory: (HardwareMap) -> Follower) =
    addComponents(PedroComponent(followerFactory))

fun NextFTCOpMode.resetPinpoint() {
    val pinpoint = hardwareMap.get(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver::class.java, "pp")
    pinpoint.resetPosAndIMU()
}

fun Pose.genVector(otherPose: Pose): Vector = Vector(hypot(otherPose.x-this.x, otherPose.y-this.y),atan2(otherPose.y-this.y,otherPose.x-this.x))
fun Vector.toPose(): Pose = Pose(this.xComponent, this.yComponent)
