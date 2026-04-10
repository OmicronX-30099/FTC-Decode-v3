@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Systems.Shooter

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.VoltageSensor
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.ServoGroup
import kotlin.math.abs
import kotlin.math.sign
@Configurable
object Hood: Subsystem {
    private val hoodServo: ServoEx = ServoEx("hood", -0.1)

    @JvmField var compensationFactor: Double = 0.0001
    @JvmField var minHoodPos: Double = 0.0
    @JvmField var maxHoodPos: Double = 1.0
    
    // Distance (inches) to Servo Position (0.0 - 1.0)
    private val table = listOf(
        40.0 to 0.4,
        60.0 to 0.5,
        80.0 to 0.6,
        100.0 to 0.7
    )

    var targetPosition: Double = 0.0

    fun update(distance: Double, velocityError: Double) {
        val basePosition = interpolate(distance)
        val compensation = velocityError * compensationFactor
        targetPosition = (basePosition - compensation).coerceIn(minHoodPos, maxHoodPos)
        hoodServo.position = targetPosition
    }

    fun reset() {
        targetPosition = 0.5
        hoodServo.position = 0.5
    }

    private fun interpolate(distance: Double): Double {
        if (table.isEmpty()) return 0.5
        if (distance <= table.first().first) return table.first().second
        if (distance >= table.last().first) return table.last().second

        for (i in 0 until table.size - 1) {
            val (d1, p1) = table[i]
            val (d2, p2) = table[i+1]
            if (distance in d1..d2) {
                val t = (distance - d1) / (d2 - d1)
                return p1 + t * (p2 - p1)
            }
        }
        return 0.5
    }

    override fun initialize() { targetPosition = 0.5 }
    fun debug(): String = "Target Position = $targetPosition \nComp Factor = $compensationFactor"
}

@Configurable
object Flywheel: Subsystem {
    private val flywheelMotor1: MotorEx = MotorEx("fwl")
    private val flywheelMotor2: MotorEx = MotorEx("fwr")
    private val flywheelMotors: MotorGroup = MotorGroup(flywheelMotor1, flywheelMotor2)
    private val voltageSensor: VoltageSensor by lazy { ActiveOpMode.hardwareMap.get(VoltageSensor::class.java, "Control Hub") }

    private val flywheelCoeffs: PSVCoeffs = PSVCoeffs(0.003, 0.08, 0.0002)
    @JvmField
    var IDLE_VELOCITY = 1500.0
    var targetVelocity: Double = 0.0
    val currentVelocity: Double get() = -flywheelMotors.velocity

    fun atTarget(): Boolean = (abs(targetVelocity - currentVelocity) <= 20.0)

    fun calculatePow(): Double = ((flywheelCoeffs.kP * (targetVelocity - currentVelocity)) + (flywheelCoeffs.kV * targetVelocity) + (flywheelCoeffs.kS * sign(targetVelocity)))
    fun update() {
        val currVoltage: Double = voltageSensor.voltage
        flywheelCoeffs.apply {
            kP = -0.0033333 * currVoltage + 0.045
            kV = -0.00003333 * currVoltage + 0.000777
        }
        flywheelMotors.power = calculatePow()
    }
    fun reset() { targetVelocity = 0.0 }
    fun debug(): String = "Target Velocity = $targetVelocity \nCurrent Velocity = ${-flywheelMotors.velocity} \nCoeffs = $flywheelCoeffs \nPower = ${flywheelMotors.power}"
}

object Turret: Subsystem {
    private val turretServo1: ServoEx = ServoEx("lt")
    private val turretServo2: ServoEx = ServoEx("ft")

    private const val GEAR_RATIO: Double = 0.9375
    private const val SERVO_RANGE: Double = 360.0

    var offset: Double = 0.0
        private set
    var targetAngle: Double = 0.0

    fun offset(by: Double) { offset += by }
    fun update() {
        turretServo1.position = (normalizeAngle300(targetAngle + offset) * (GEAR_RATIO / SERVO_RANGE) + 0.495) //+ 0.00130571*2.0
        turretServo2.position = (normalizeAngle300(targetAngle + offset) * (GEAR_RATIO / SERVO_RANGE) + 0.505) //- 0.00130571*2.0
    }
    fun reset() { offset = 0.0; targetAngle = 0.0 }
    fun debug(): String = "Target Angle = $targetAngle \nOffset = $offset \nCurrent Position = ${turretServo1.position - 0.00130571*2.0}"
}

internal fun normalizeAngle(angDeg: Double): Double {
    var a = angDeg
    while (a < 0.0) { a += 360.0 }
    if (a > 180.0) { a -= 360.0 }
    return a
}

internal fun normalizeAngle300(angDeg: Double): Double {
    var normalized = normalizeAngle(angDeg)
    return (normalized.coerceIn(-180.0,180.0))
}

enum class FlywheelState {
    PREDICTIVE_AUTO_AIM,
    AUTO_AIM,
    MANUAL
}

data class PSVCoeffs ( @JvmField var kP: Double, @JvmField var kS: Double, @JvmField var kV: Double)