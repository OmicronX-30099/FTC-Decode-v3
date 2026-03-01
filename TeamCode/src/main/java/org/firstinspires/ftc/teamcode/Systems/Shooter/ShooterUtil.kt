@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Systems.Shooter

import com.qualcomm.robotcore.hardware.VoltageSensor
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.ServoGroup
import kotlin.math.abs
import kotlin.math.sign

object Flywheel: Subsystem {
    private val flywheelMotor1: MotorEx = MotorEx("fwt")
    private val flywheelMotor2: MotorEx = MotorEx("fwb")
    private val flywheelMotors: MotorGroup = MotorGroup(flywheelMotor1, flywheelMotor2)
    private val voltageSensor: VoltageSensor by lazy { ActiveOpMode.hardwareMap.get(VoltageSensor::class.java, "Control Hub") }

    private val flywheelCoeffs: PSVCoeffs = PSVCoeffs(0.003, 0.08, 0.0002)

    const val IDLE_VELOCITY = 1500.0
    var targetVelocity: Double = 0.0

    fun atTarget(): Boolean = (abs(targetVelocity - flywheelMotors.velocity) <= 20.0)

    fun calculatePow(): Double = ((flywheelCoeffs.kP * (targetVelocity - flywheelMotors.velocity)) + (flywheelCoeffs.kV * targetVelocity) + (flywheelCoeffs.kS * sign(targetVelocity)))
    fun update() {
        val currVoltage: Double = voltageSensor.voltage
        flywheelCoeffs.apply {
            kP = -0.0033333 * currVoltage + 0.045
            kV = -0.00003333 * currVoltage + 0.000777
        }
        flywheelMotors.power = calculatePow()
    }
    fun reset() { targetVelocity = 0.0 }
    fun debug(): String = "Target Velocity = $targetVelocity \nCurrent Velocity = ${flywheelMotors.velocity} \nCoeffs = $flywheelCoeffs \nPower = ${flywheelMotors.power}"
}

object Turret: Subsystem {
    private val turretServo1: ServoEx = ServoEx("lt")
    private val turretServo2: ServoEx = ServoEx("ft")
    private val turretServos: ServoGroup = ServoGroup(turretServo1, turretServo2)

    private const val GEAR_RATIO: Double = 0.9375
    private const val SERVO_RANGE: Double = 360.0

    var offset: Double = 0.0
        private set
    var targetAngle: Double = 0.0

    fun offset(by: Double) { offset += by }
    fun update() { turretServos.position = (normalizeAngle(targetAngle + offset) * (GEAR_RATIO / SERVO_RANGE) + 0.5) }
    fun reset() { offset = 0.0; targetAngle = 0.0 }
    fun debug(): String = "Target Angle = $targetAngle \nOffset = $offset \nCurrent Position = ${turretServos.position}"
}

internal fun normalizeAngle(angDeg: Double): Double {
    var a = angDeg
    while (a < 0.0) { a += 360.0 }
    if (a > 180.0) { a -= 360.0 }
    return a
}

enum class FlywheelState {
    PREDICTIVE_AUTO_AIM,
    AUTO_AIM,
    MANUAL
}

data class PSVCoeffs ( @JvmField var kP: Double, @JvmField var kS: Double, @JvmField var kV: Double)