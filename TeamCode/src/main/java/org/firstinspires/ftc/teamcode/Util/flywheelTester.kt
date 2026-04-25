package org.firstinspires.ftc.teamcode.Util

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.VoltageSensor
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.Systems.Shooter.PSVCoeffs
import kotlin.math.sign

@Configurable
@TeleOp
class flywheelTester: NextFTCOpMode(){
    init {
        includePedro(Constants::createFollower)
    }
    companion object {
        @JvmField
        var targetVelocity = 0.0
        @JvmField
        var turret = 0.5
        @JvmField
        var hoodpose = 0.92
        @JvmField
        var flywheelCoeffs: PSVCoeffs = PSVCoeffs(0.003, 0.09, 0.0004)
    }
    private val flywheelMotor1: MotorEx = MotorEx("fwl").reversed()
    private val flywheelMotor2: MotorEx = MotorEx("fwr").reversed()
    private val transferMotor: MotorEx = MotorEx("t").reversed()
    private val intakeMotor: MotorEx = MotorEx("i").reversed()
    private val turretServo1: ServoEx = ServoEx("lt")
    private val turretServo2: ServoEx = ServoEx("ft")
    private val hood: ServoEx = ServoEx("hood")
    private val flywheelMotors: MotorGroup = MotorGroup(flywheelMotor1, flywheelMotor2)
    private val voltageSensor: VoltageSensor by lazy { ActiveOpMode.hardwareMap.get(VoltageSensor::class.java, "Control Hub") }


    var count  = 0

    val joinedTelemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)

    fun calculatePow(): Double = ((flywheelCoeffs.kP * (targetVelocity - flywheelMotors.velocity)) + (flywheelCoeffs.kV * targetVelocity) + (flywheelCoeffs.kS * sign(targetVelocity)))

    override fun onStartButtonPressed() {
        follower.setStartingPose(Pose((141.5-23.875-6.23),(141.5-14.0+6.7),Math.toRadians(90.0)))
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
        /*flywheelCoeffs.apply {
            kP = -0.0033333 * currVoltage + 0.045
            kV = -0.00003333 * currVoltage + 0.000777
        }*/
        flywheelMotors.power = calculatePow()
        turretServo1.position = turret
        turretServo2.position = turret
        hood.position = hoodpose
        joinedTelemetry.addData("Pose: ", "${follower.pose.x}, ${follower.pose.y}, ${Math.toDegrees(follower.pose.heading)}")
        joinedTelemetry.addData("distance: ", follower.pose.distanceFrom(Alliance.RED.flywheelGoalPose))
        joinedTelemetry.addData("Target velocity: ", targetVelocity)
        joinedTelemetry.addData("Curr velocity1: ", flywheelMotor1.velocity)
        joinedTelemetry.addData("Curr velocity2: ", flywheelMotor2.velocity)
        joinedTelemetry.update()
    }
}