package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.hardware.driving.DriverControlledCommand
import org.firstinspires.ftc.teamcode.Constants.PedroConstants
import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.BIM
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.Transfer
import org.firstinspires.ftc.teamcode.Systems.Load
import org.firstinspires.ftc.teamcode.Systems.Shooter
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.currAlliance
import org.firstinspires.ftc.teamcode.Util.currStartPose
import org.firstinspires.ftc.teamcode.Util.includePedro

@TeleOp(name = "Blue TeleOp - DP", group = "Blue TeleOps")
class DPTeleOpBlue: NextFTCOpMode() {
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
        currAlliance = Alliance.BLUE
        drivetrain.schedule()
        follower.setStartingPose(currStartPose)
        Gamepads.gamepad1.rightTrigger.greaterThan(0.0)
            .whenBecomesTrue {
                Transfer.transfer(0.35)
                Transfer.intake(1.0)
            }
            .whenBecomesFalse {
                Transfer.transfer(0.0)
                Transfer.intake(0.0)
            }
        Gamepads.gamepad1.leftTrigger.greaterThan(0.0).and(Gamepads.gamepad1.rightTrigger.atMost(0.0))
            .whenBecomesTrue {
                Transfer.transfer(-1.0)
                Transfer.intake(-1.0)
            }
            .whenBecomesFalse {
                Transfer.transfer(0.0)
                Transfer.intake(0.0)
            }

        Gamepads.gamepad1.rightBumper
            .whenBecomesTrue(Load.shootTripleCommand)
        Gamepads.gamepad1.leftBumper
            .whenBecomesTrue {
                drivetrain.scalar = (drivetrain.scalar + 0.6) % 1.2
            }
        Gamepads.gamepad1.dpadUp
            .whenBecomesTrue { BIM.raise() }
        Gamepads.gamepad1.dpadDown
            .whenBecomesTrue { BIM.lower() }
    }

    override fun onUpdate() {
        Shooter.update()
        telemetry.update()
    }
}