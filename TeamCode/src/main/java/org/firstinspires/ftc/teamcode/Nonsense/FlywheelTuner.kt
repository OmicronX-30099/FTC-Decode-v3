package org.firstinspires.ftc.teamcode.Nonsense

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.Flywheel
import org.firstinspires.ftc.teamcode.Util.addSubsystems

@TeleOp(name = "Flywheel Tuner")
class FlywheelTuner: NextFTCOpMode() {
    init {
        addSubsystems(Flywheel)
    }

    companion object {
        @JvmField
        var flywheelVel: Double = 0.0
    }

    override fun onUpdate() {
        Flywheel.flywheelTarget = flywheelVel
        Flywheel.update()
        telemetry.addData("x",follower.pose.x)
        telemetry.addData("y",follower.pose.y)
        telemetry.addData("h",follower.pose.heading)
        telemetry.update()
    }
}