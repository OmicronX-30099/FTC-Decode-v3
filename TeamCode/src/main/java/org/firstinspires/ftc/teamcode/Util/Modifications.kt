package org.firstinspires.ftc.teamcode.Util

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.fateweaver.FateComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.ftc.components.LoopTimeComponent
import kotlin.math.cos
import kotlin.math.sin

// NextFTCOpMode extension functions
fun NextFTCOpMode.addAll() = addComponents(BulkReadComponent, BindingsComponent, LoopTimeComponent())
fun NextFTCOpMode.subsystems(vararg subsystems: Subsystem) = addComponents(SubsystemComponent(*subsystems))
fun NextFTCOpMode.enablePedro(followerSupp: (HardwareMap) -> Follower) = addComponents(PedroComponent(followerSupp))
fun NextFTCOpMode.enableLog() = addComponents(FateComponent)

// Pedro Extension functions
fun Vector.asPose(): Pose = Pose(this.xComponent, this.yComponent)
fun Follower.fullVelocity(): Pose = (this.velocity.asPose() + Pose(0.0,0.0,this.angularVelocity))
fun Pose.offset(yOffset: Double) =
    (
        this + Pose(
            yOffset * cos(this.heading),
            yOffset * sin(this.heading)
        )
    )

infix fun <T> T.exec(task: T.() -> Unit): T = this.apply(task)
infix fun <T> T.execute(task: T.() -> Unit): T = this.apply(task)
infix fun <T> T.and(task: (T) -> Unit): T = this.also(task)