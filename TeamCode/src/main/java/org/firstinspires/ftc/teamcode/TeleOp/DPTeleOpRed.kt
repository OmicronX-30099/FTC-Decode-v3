package org.firstinspires.ftc.teamcode.TeleOp

import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.hardware.driving.DriverControlledCommand
import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.Final.Subsystems.ShooterSystem
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.TransferSubsystem
import org.firstinspires.ftc.teamcode.Systems.PassiveSystem
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.FlywheelState
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.TurretState
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.currAlliance
import kotlin.math.PI

@TeleOp(name = "Red TeleOp - DP")
class DPTeleOpRed: NextFTCOpMode() {
    init {
        addSubsystems(ShooterSystem, IntakeSubsystem)
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
        follower.setStartingPose(Pose(72.0,72.0,PI/2))
        Gamepads.gamepad1.rightTrigger.greaterThan(0.0)
            .whenBecomesTrue {
                IntakeSubsystem.intake(1.0)
                TransferSubsystem.transfer(0.35)
            }
            .whenBecomesFalse {
                IntakeSubsystem.intake(0.0)
                TransferSubsystem.transfer(0.0)
            }
        Gamepads.gamepad1.leftTrigger.greaterThan(0.0).and(Gamepads.gamepad1.rightTrigger.inRange(0.0..0.0))
            .whenBecomesTrue {
                IntakeSubsystem.intake(-1.0)
                TransferSubsystem.transfer(-1.0)
            }
            .whenBecomesFalse {
                IntakeSubsystem.intake(0.0)
                TransferSubsystem.transfer(0.0)
            }
        Gamepads.gamepad1.rightBumper
            .whenBecomesTrue(PassiveSystem.shootTripleCommand)
        Gamepads.gamepad1.leftBumper
            .whenBecomesTrue(PassiveSystem.shootSingular)
        Gamepads.gamepad1.cross
            .whenBecomesTrue {
                ShooterSystem.currTurretState = TurretState.MANUAL
                ShooterSystem.currFlywheelState = FlywheelState.MANUAL
            }
        Gamepads.gamepad1.triangle
            .whenBecomesTrue {
                ShooterSystem.currTurretState = TurretState.AUTO_AIM
                ShooterSystem.currFlywheelState = FlywheelState.AUTO_AIM
            }
        Gamepads.gamepad2.rightBumper
            .whenBecomesTrue {
                ShooterSystem.currTurretState = TurretState.MANUAL
                ShooterSystem.currFlywheelState = FlywheelState.MANUAL
            }
        Gamepads.gamepad2.leftBumper
            .whenBecomesTrue {
                ShooterSystem.currTurretState = TurretState.AUTO_AIM
                ShooterSystem.currFlywheelState = FlywheelState.AUTO_AIM
            }
        Gamepads.gamepad2.dpadLeft
            .whenBecomesTrue {
                ShooterSystem.currTurretState = TurretState.MANUAL
                ShooterSystem.moveTurretBy(5.0)
            }
        Gamepads.gamepad2.dpadRight
            .whenBecomesTrue {
                ShooterSystem.currTurretState = TurretState.MANUAL
                ShooterSystem.moveTurretBy(-5.0)
            }
        Gamepads.gamepad1.dpadUp
            .whenBecomesTrue {
                ShooterSystem.increaseFlywheelVelocityBy(40.0)
                ShooterSystem.currFlywheelState = FlywheelState.MANUAL
            }
        Gamepads.gamepad1.dpadDown
            .whenBecomesTrue {
                ShooterSystem.increaseFlywheelVelocityBy(-40.0)
                ShooterSystem.currFlywheelState = FlywheelState.MANUAL
            }
    }

    override fun onUpdate() {
        ShooterSystem.updateShooter()
    }
}