package org.firstinspires.ftc.teamcode.Nonsense

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.Constants.PedroConstants
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.Flywheel
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.includePedro

@Configurable
@TeleOp(name = "Flywheel Tuner")
class FlywheelTuner: NextFTCOpMode() {
    init {
        addSubsystems(Flywheel)
        includePedro(PedroConstants::createFollower)
    }

    companion object {
        var startPose: Pose = Pose(0.0,0.0)
        @JvmField
        var flywheelVel: Double = 0.0
    }

    override fun onStartButtonPressed() {
        follower.setStartingPose(startPose)
    }

    override fun onUpdate() {
        Flywheel.flywheelTarget = flywheelVel
        Flywheel.update()
        telemetry.addData("x",follower.pose.x)
        telemetry.addData("y",follower.pose.y)
        telemetry.addData("h",follower.pose.heading)
        telemetry.update()
    }

    override fun onStop() {
        startPose = follower.pose
    }
}