package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.hardware.driving.DriverControlledCommand
import org.firstinspires.ftc.teamcode.Constants.PedroConstants
import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.Systems.Load
import org.firstinspires.ftc.teamcode.Systems.Shooter
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.currAlliance
import org.firstinspires.ftc.teamcode.Util.currStartPose
import org.firstinspires.ftc.teamcode.Util.includePedro

@TeleOp(name = "Red TeleOp - DP", group = "Red TeleOps")
class DPTeleOpRed: NextFTCOpMode() {
    init {
        addSubsystems(Shooter, Load)
        includePedro(PedroConstants::createFollower)
    }

    val drivetrain: DriverControlledCommand by lazy {
        PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY,
            -Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            true
        )
    }

    override fun onStartButtonPressed() {
        currAlliance = Alliance.RED
        drivetrain.schedule()
        follower.setStartingPose(currStartPose)

    }

    override fun onUpdate() {
        Shooter.update()

        telemetry.addData("x",follower.pose.x)

        telemetry.addData("y",follower.pose.y)

        telemetry.addData("h",follower.pose.heading)
        telemetry.update()
    }
}