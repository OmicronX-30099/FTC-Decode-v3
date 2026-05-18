@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.hardware.driving.DriverControlledCommand
import org.firstinspires.ftc.teamcode.Systems.Load.BreakBeam
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.Systems.Load.Load
import org.firstinspires.ftc.teamcode.Systems.Load.Rollers
import org.firstinspires.ftc.teamcode.Systems.Shooter.Flywheel
import org.firstinspires.ftc.teamcode.Systems.Shooter.FlywheelState
import org.firstinspires.ftc.teamcode.Systems.Shooter.Shooter
import org.firstinspires.ftc.teamcode.Systems.Shooter.Shooter.shooterMethod
import org.firstinspires.ftc.teamcode.Systems.Shooter.ShooterMethod
import org.firstinspires.ftc.teamcode.Systems.Shooter.Turret
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.includePedro

@TeleOp(name = "Red TeleOp", group = "Standard TeleOp")
class RedTeleOp: NextFTCOpMode() {
    init {
        addSubsystems(Load, Shooter)
        includePedro(Constants::createFollower)
    }

    private val headingLock = HeadingLockTurnController()
    private var lastTelemetryUpdateTime = 0L

    val drivetrain: DriverControlledCommand by lazy {
        PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY.map { it * 0.8 },
            -Gamepads.gamepad1.leftStickX,
            { headingLock.turnPower(ActiveOpMode.gamepad1.right_stick_x.toDouble()) },
            true
        )
    }

    /*val drivetrain: HeadingLockDriveCommand by lazy {
        HeadingLockDriveCommand(
            -Gamepads.gamepad1.leftStickY,
            -Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            // Change this value based on teleop, what u wanna do, etc.
            // U could also do a supplier, which will allow it to auto update the heading goal
            Math.toRadians(30.0)
        )
    }*/

    override fun onInit() { Shooter.reset() }

    override fun onStartButtonPressed() {
        shooterMethod = ShooterMethod.REGRESSION
        Shooter.flywheelState = FlywheelState.PREDICTIVE_AUTO_AIM
        ROBOT.currStage = Stage.TELEOP
        ROBOT.currAlliance = Alliance.RED
        follower.setStartingPose(ROBOT.teleopStartPose)
        drivetrain.schedule()
        Gamepads.gamepad1 .apply {
            rightTrigger.greaterThan(0.0)
                .whenBecomesTrue { Rollers.run(1.0,0.67) }
                .whenBecomesFalse { Rollers.stop() }
            leftTrigger.greaterThan(0.0).and(rightTrigger.inRange(0.0..0.0))
                .whenBecomesTrue { Rollers.run(-1.0,-1.0) }
                .whenBecomesFalse { Rollers.stop() }
            rightBumper
                .whenBecomesTrue ( Load.shootTripleCommand )
            leftBumper
                .toggleOnBecomesTrue()
                .whenBecomesTrue { drivetrain.scalar = 0.2 }
                .whenBecomesFalse { drivetrain.scalar = 1.0 }
            circle
                .whenBecomesTrue { Shooter.flywheelManual() }
            cross
                .whenBecomesTrue { follower.pose = ROBOT.currAlliance.resetPoses.resetPose1 }
            square
                .whenBecomesTrue { headingLock.enableIfTurnStickCentered(ActiveOpMode.gamepad1.right_stick_x.toDouble()) }

            /*square
                .whenBecomesTrue { drivetrain.lockHeading = true }*/

            triangle
                .toggleOnBecomesTrue()
                .whenBecomesTrue { Shooter.enableAutoAim() }
                .whenBecomesFalse { Shooter.enablePredictive() }
        }
        Gamepads.gamepad2 .apply {
            triangle
                .whenBecomesTrue { follower.pose = ROBOT.currAlliance.resetPoses.resetPose2 }
            square
                .whenBecomesTrue { follower.pose = ROBOT.currAlliance.resetPoses.resetPose3 }
        }
        Gamepads .apply {
            gamepad1.dpadLeft.or(gamepad2.dpadLeft)
                .whenBecomesTrue { Turret.offset(1.0) }
            gamepad1.dpadRight.or(gamepad2.dpadRight)
                .whenBecomesTrue { Turret.offset(-1.0) }
            gamepad1.dpadUp.or(gamepad2.dpadUp)
                .whenBecomesTrue {
                    if (Shooter.flywheelState == FlywheelState.MANUAL) {
                        Flywheel.targetVelocity += 20.0
                    } else {
                        Flywheel.offsetVelocity(20.0)
                    }
                }
            gamepad1.dpadDown.or(gamepad2.dpadDown)
                .whenBecomesTrue {
                    if (Shooter.flywheelState == FlywheelState.MANUAL) {
                        Flywheel.targetVelocity -= 20.0
                    } else {
                        Flywheel.offsetVelocity(-20.0)
                    }
                }
        }
    }

    override fun onUpdate() {
        val currentBallCount = BreakBeam.refreshBallCount()
        Shooter.update()
        Rollers.update(currentBallCount)

        val now = System.currentTimeMillis()
        if (now - lastTelemetryUpdateTime >= 200) {
            telemetry.run {
                addData("Follower", follower.pose)
                addData("balls: ", currentBallCount)
                addData("Velocity", follower.velocity)
                addLine(Shooter.debug())
                update()
            }
            lastTelemetryUpdateTime = now
        }
    }
}
