package org.firstinspires.ftc.teamcode.Util

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.panels.Panels
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.VoltageSensor
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.Constants.PedroConstants
import org.firstinspires.ftc.teamcode.Systems.Load.Rollers
import org.firstinspires.ftc.teamcode.Systems.Shooter.Flywheel
import org.firstinspires.ftc.teamcode.Systems.Shooter.PSVCoeffs
import kotlin.math.sign

@Configurable
@TeleOp
class flywheelTester: NextFTCOpMode(){
    init {
        includePedro(PedroConstants::createFollower)
    }
    companion object {
        @JvmField
        var targetVelocity = 0.0
        @JvmField
        var servoPos = 0.5
    }
    private val flywheelMotor1: MotorEx = MotorEx("fwl")
    private val flywheelMotor2: MotorEx = MotorEx("fwr")
    private val transferMotor: MotorEx = MotorEx("t").reversed()
    private val intakeMotor: MotorEx = MotorEx("i")
    private val turretServo1: ServoEx = ServoEx("lt")
    private val turretServo2: ServoEx = ServoEx("ft")
    private val flywheelMotors: MotorGroup = MotorGroup(flywheelMotor1, flywheelMotor2)
    private val voltageSensor: VoltageSensor by lazy { ActiveOpMode.hardwareMap.get(VoltageSensor::class.java, "Control Hub") }

    private val flywheelCoeffs: PSVCoeffs = PSVCoeffs(0.003, 0.08, 0.0002)
    var count  = 0

    val joinedTelemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)

    fun calculatePow(): Double = ((flywheelCoeffs.kP * (targetVelocity - flywheelMotors.velocity)) + (flywheelCoeffs.kV * targetVelocity) + (flywheelCoeffs.kS * sign(targetVelocity)))

    override fun onStartButtonPressed() {
        follower.setStartingPose(Pose(79.75, 7.5, Math.toRadians(90.0)))
    }
    override fun onUpdate() {
        if (gamepad1.rightBumperWasPressed()) {
            count += 1
            if (count % 2 == 1) {
                intakeMotor.power = 1.0
                transferMotor.power = 1.0
            } else {
                intakeMotor.power = 0.0
                transferMotor.power = 0.0
            }
        }
        val currVoltage: Double = voltageSensor.voltage
        flywheelCoeffs.apply {
            kP = -0.0033333 * currVoltage + 0.045
            kV = -0.00003333 * currVoltage + 0.000777
        }
        flywheelMotors.power = calculatePow()
        turretServo1.position = servoPos
        turretServo2.position = servoPos
        joinedTelemetry.addData("Pose: ", "${follower.pose.x}, ${follower.pose.y}, ${Math.toDegrees(follower.pose.heading)}")
        joinedTelemetry.addData("distance: ", follower.pose.distanceFrom(Alliance.RED.flywheelGoalPose))
        joinedTelemetry.addData("Target velocity: ", targetVelocity)
        joinedTelemetry.addData("Curr velocity: ", flywheelMotors.velocity)
        joinedTelemetry.update()
    }
}