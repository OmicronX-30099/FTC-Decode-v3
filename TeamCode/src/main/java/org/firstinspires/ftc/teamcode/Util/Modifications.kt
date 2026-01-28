package org.firstinspires.ftc.teamcode.Util

import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.nextftc.bindings.Button
import dev.nextftc.bindings.button
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.ftc.components.LoopTimeComponent
import org.firstinspires.ftc.teamcode.Enums.Alliance

fun NextFTCOpMode.addSubsystems(vararg subsystems: Subsystem) {
    addComponents(BindingsComponent, BulkReadComponent, LoopTimeComponent(), SubsystemComponent(*subsystems))
}
fun NextFTCOpMode.includePedro(followerFactory: (HardwareMap) -> Follower) {
    addComponents(PedroComponent(followerFactory))
}

var currAlliance: Alliance = Alliance.BLUE