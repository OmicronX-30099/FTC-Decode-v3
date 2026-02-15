package org.firstinspires.ftc.teamcode

import com.pedropathing.geometry.Pose
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.Constants.PedroConstants
import kotlin.math.PI

class LocalizationTester: NextFTCOpMode() {
    init {
        addComponents(
            BindingsComponent,
            BulkReadComponent,
            PedroComponent(PedroConstants::createFollower)
        )
    }

    val drivetrain by lazy {
        PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY,
            -Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            true
        )
    }

    override fun onStartButtonPressed() {
        follower.setStartingPose(Pose(0.0,0.0,PI/2.0))
        drivetrain.schedule()
    }

    override fun onUpdate() {
        telemetry.run {
            addLine("X: ${follower.pose.x}")
            addLine("Y: ${follower.pose.y}")
            addLine("H: ${follower.pose.heading}")
            update()
        }
    }
}