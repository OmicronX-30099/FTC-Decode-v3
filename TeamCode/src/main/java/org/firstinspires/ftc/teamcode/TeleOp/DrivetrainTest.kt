@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.hardware.driving.DriverControlledCommand
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.Util.includePedro
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import kotlin.math.abs

@TeleOp(name = "Drivetrain Test", group = "Testing")
class DrivetrainTest: NextFTCOpMode() {
    init {
        addSubsystems()
        includePedro(Constants::createFollower)
    }

    private val intakeMotor: MotorEx = MotorEx("i").reversed()
    private val transferMotor: MotorEx = MotorEx("t").reversed()

    val drivetrain: DriverControlledCommand by lazy {
        PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY,
            -Gamepads.gamepad1.leftStickX,
            (-Gamepads.gamepad1.rightStickX).map { (it * abs(it) + it) / 2.0 },
            true
        )
    }

    override fun onStartButtonPressed() {
        drivetrain.schedule()
        Gamepads.gamepad1 .apply {
            rightTrigger.greaterThan(0.0)
                .whenBecomesTrue { intakeMotor.power = 1.0; transferMotor.power = 1.0 }
                .whenBecomesFalse { intakeMotor.power = 0.0; transferMotor.power = 0.0 }
            leftTrigger.greaterThan(0.0).and(rightTrigger.inRange(0.0..0.0))
                .whenBecomesTrue { intakeMotor.power = -1.0; transferMotor.power = -1.0 }
                .whenBecomesFalse { intakeMotor.power = 0.0; transferMotor.power = 0.0 }
        }
    }

    override fun onUpdate() {

    }
}
