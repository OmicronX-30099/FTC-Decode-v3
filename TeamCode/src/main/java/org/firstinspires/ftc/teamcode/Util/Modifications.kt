package org.firstinspires.ftc.teamcode.Util

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.ftc.components.LoopTimeComponent
import org.firstinspires.ftc.teamcode.Enums.Alliance
import kotlin.math.PI

fun NextFTCOpMode.addSubsystems(vararg subsystems: Subsystem) {
    addComponents(BindingsComponent, BulkReadComponent, LoopTimeComponent(), SubsystemComponent(*subsystems))
}
fun NextFTCOpMode.includePedro(followerFactory: (HardwareMap) -> Follower) {
    addComponents(PedroComponent(followerFactory))
}

var currAlliance: Alliance = Alliance.BLUE
var currStartPose: Pose = Pose(72.0,72.0, PI/2.0)