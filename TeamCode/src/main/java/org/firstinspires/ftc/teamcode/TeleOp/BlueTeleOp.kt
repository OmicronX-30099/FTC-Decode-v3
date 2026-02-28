@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.hardware.driving.DriverControlledCommand
import org.firstinspires.ftc.teamcode.Constants.PedroConstants
import org.firstinspires.ftc.teamcode.Systems.Load.Load
import org.firstinspires.ftc.teamcode.Systems.Load.Rollers
import org.firstinspires.ftc.teamcode.Systems.Shooter.Flywheel
import org.firstinspires.ftc.teamcode.Systems.Shooter.FlywheelState
import org.firstinspires.ftc.teamcode.Systems.Shooter.Shooter
import org.firstinspires.ftc.teamcode.Systems.Shooter.Turret
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.includePedro

@TeleOp(name = "Blue TeleOp", group = "Standard TeleOp")
class BlueTeleOp: NextFTCOpMode() {
    init {
        addSubsystems(Load, Shooter)
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

    override fun onInit() { Shooter.reset() }

    override fun onStartButtonPressed() {
        ROBOT.currStage = Stage.TELEOP
        ROBOT.currAlliance = Alliance.BLUE
        follower.setStartingPose(ROBOT.teleopStartPose)
        drivetrain.schedule()
        Gamepads.gamepad1.rightTrigger.greaterThan(0.0)
            .whenBecomesTrue { Rollers.run(0.35,1.0) }
            .whenBecomesFalse { Rollers.stop() }
        Gamepads.gamepad1.leftTrigger.greaterThan(0.0).and(Gamepads.gamepad1.rightTrigger.inRange(0.0..0.0))
            .whenBecomesTrue { Rollers.run(-1.0,-1.0) }
            .whenBecomesFalse { Rollers.stop() }
        Gamepads.gamepad1.rightBumper
            .whenBecomesTrue ( Load.shootTripleCommand )
        Gamepads.gamepad1.leftBumper
            .toggleOnBecomesTrue()
            .whenBecomesTrue { drivetrain.scalar = 0.2 }
            .whenBecomesFalse { drivetrain.scalar = 1.0 }
        Gamepads.gamepad1.circle
            .toggleOnBecomesTrue()
            .whenBecomesTrue { Shooter.flywheelManual() }
            .whenBecomesTrue { Shooter.flywheelAutoAim() }
        Gamepads.gamepad1.cross
            .whenBecomesTrue { follower.pose = ROBOT.currAlliance.resetPoses.resetPose1 }
        Gamepads.gamepad2.triangle
            .whenBecomesTrue { follower.pose = ROBOT.currAlliance.resetPoses.resetPose2 }
        Gamepads.gamepad2.square
                .whenBecomesTrue { follower.pose = ROBOT.currAlliance.resetPoses.resetPose3 }
        Gamepads.gamepad1.dpadLeft.or(Gamepads.gamepad2.dpadLeft)
            .whenBecomesTrue { Turret.offset(1.0) }
        Gamepads.gamepad1.dpadRight.or(Gamepads.gamepad2.dpadRight)
            .whenBecomesTrue { Turret.offset(-1.0) }
        Gamepads.gamepad1.dpadUp.or(Gamepads.gamepad2.dpadUp)
            .whenBecomesTrue {
                if (Shooter.flywheelState == FlywheelState.MANUAL) {
                    Flywheel.targetVelocity += 40.0
                }
            }
        Gamepads.gamepad1.dpadDown.or(Gamepads.gamepad2.dpadDown)
            .whenBecomesTrue {
                if (Shooter.flywheelState == FlywheelState.MANUAL) {
                    Flywheel.targetVelocity -= 40.0
                }
            }
        }
    }

    override fun onUpdate() {
        Shooter.update()
        telemetry.run {
            addData("Follower", follower.pose)
            addLine(Shooter.debug())
            update()
        }
    }
}