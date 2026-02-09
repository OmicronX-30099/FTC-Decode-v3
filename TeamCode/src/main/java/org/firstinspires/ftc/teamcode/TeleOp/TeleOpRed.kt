package org.firstinspires.ftc.teamcode.TeleOp

import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.hardware.driving.DriverControlledCommand
import org.firstinspires.ftc.teamcode.Constants.PedroConstants
import org.firstinspires.ftc.teamcode.ShooterSubsystems.FlywheelState
import org.firstinspires.ftc.teamcode.Systems.Load
import org.firstinspires.ftc.teamcode.Systems.Rollers
import org.firstinspires.ftc.teamcode.Systems.Shooter
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.includePedro
import kotlin.math.PI

@TeleOp(name = "Red TeleOp", group = "TeleOp")
class TeleOpRed: NextFTCOpMode() {
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
        ROBOT.currAlliance = Alliance.RED
        ROBOT.currStage = Stage.TELEOP
        follower.setStartingPose(Pose(70.5,70.5,PI/2))
        drivetrain.schedule()

        Gamepads.gamepad1.rightTrigger.greaterThan(0.0)
            .whenBecomesTrue { Rollers.run(0.35,1.0) }
            .whenBecomesFalse { Rollers.run(0.0,0.0) }
        Gamepads.gamepad1.leftTrigger.greaterThan(0.0).and(Gamepads.gamepad1.rightTrigger.inRange(0.0..0.0))
            .whenBecomesTrue { Rollers.run(-1.0,-1.0) }
            .whenBecomesFalse { Rollers.run(0.0,0.0) }
        Gamepads.gamepad1.rightBumper
            .whenBecomesTrue ( Load.shootTripleCommand )
        Gamepads.gamepad1.cross.or(Gamepads.gamepad1.rightBumper)
            .whenBecomesTrue {
                if (Shooter.flywheelState == FlywheelState.AUTO_AIM) { Shooter.flywheelState = FlywheelState.IDLE }
                else { Shooter.flywheelState = FlywheelState.AUTO_AIM }
            }
        Gamepads.gamepad1.dPadUp.or(Gamepads.gamepad2.dPadUp)
            .whenBecomesTrue {  }
        Gamepads.gamepad1.dpadDown.or(Gamepads.gamepad2.dPadDown)
            .whenBecomesTrue {  }
        Gamepads.gamepad2.dPadLeft
            .whenBecomesTrue { Shooter.moveTurretBy(5.0) }
        Gamepads.gamepad2.dPadRight
            .whenBecomesTrue { Shooter.moveTurretBy(-5.0) }
        Gamepads.gamepad2.square
            .whenBecomesTrue { follower.pose = ROBOT.currAlliance.resetPoses.resetPose2 }
        Gamepads.gamepad2.triangle
            .whenBecomesTrue { follower.pose = ROBOT.currAlliance.resetPoses.resetPose3 }
        Gamepads.gamepad2.circle
            .whenBecomesTrue { follower.pose = ROBOT.currAlliance.resetPoses.resetPose4 }
        Gamepads.gamepad2.cross
            .whenBecomesTrue { follower.pose = ROBOT.currAlliance.resetPoses.resetPose5 }
        Gamepads.gamepad1.square
            .whenBecomesTrue { follower.pose = ROBOT.currAlliance.resetPoses.resetPose1 }
    }

    override fun onUpdate() {
        Shooter.update()
        telemetry.update()
    }
}
