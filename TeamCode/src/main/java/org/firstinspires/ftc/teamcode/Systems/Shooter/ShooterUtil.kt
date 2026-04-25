@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Systems.Shooter

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.VoltageSensor
import dev.frozenmilk.util.graph.rule.dependsOn
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

    @JvmField var compensationFactor: Double = 0.000
    @JvmField var minHoodPos: Double = 0.20
    @JvmField var maxHoodPos: Double = 0.96
    
    @JvmField var servoAt15Deg: Double = 0.92
    @JvmField var servoAt35Deg: Double = 0.3274
    
    // Distance (inches) to Servo Position (0.0 - 1.0)
    private val table = listOf(
        30.700163 to 0.96,
        35.362409 to 0.92,
        40.354678 to 0.84,
        45.918406 to 0.65,
        50.916598 to 0.55,
        55.339859 to 0.50,
        60.336556 to 0.45,
        66.155121 to 0.40,
        70.586826 to 0.35,
        75.581082 to 0.30,
        80.576051 to 0.25,
        85.571607 to 0.20,
        90.567654 to 0.20,
        95.501309 to 0.20,
        100.43157 to 0.20,
        105.3684 to 0.20,
        110.69101 to 0.20,
        115.42313 to 0.20,
        121.1301 to 0.20,
        125.26971 to 0.20,
        130.86061 to 0.20,
        135.83998 to 0.20,
        140.29433 to 0.20,
        146.12495 to 0.20,
        150.58054 to 0.20,
        156.01442 to 0.20,
        160.77469 to 0.20
    )

    var targetPosition: Double = 0.0

    fun updateRegression(distance: Double, velocityError: Double) {
        val basePosition = interpolate(distance)
        val compensation = velocityError * compensationFactor
        targetPosition = (basePosition - compensation).coerceIn(minHoodPos, maxHoodPos)
        hoodServo.position = targetPosition
    }

    fun updatePhysics(targetAngle: Double, velocityError: Double) {
        // Map targetAngle [15, 35] to servo range
        val t = (targetAngle - 15.0) / (35.0 - 15.0)
        val basePosition = servoAt15Deg + t * (servoAt35Deg - servoAt15Deg)
        
        val compensation = velocityError * compensationFactor
        targetPosition = (basePosition - compensation).coerceIn(minHoodPos, maxHoodPos)
        hoodServo.position = targetPosition
    }

    fun reset() {
        targetPosition = 0.5
        //hoodServo.position = 0.5
    }

    private fun interpolate(distance: Double): Double {
        if (table.isEmpty()) return 0.20
        if (distance <= table.first().first) return table.first().second
        if (distance >= table.last().first) return table.last().second

        var value = 0.20
        for (i in 0 until table.size - 1) {
            val (d1, p1) = table[i]
            val (d2, p2) = table[i+1]
            if (distance in d1..d2) {
                val t = (distance - d1) / (d2 - d1)
                value = p1 + t * (p2 - p1)
                break
            }
        }
        return value
    }

    fun debug(): String = "Target Position = $targetPosition \nComp Factor = $compensationFactor"
}

object ShooterLights: Subsystem {
    val shooterRGB: ServoEx = ServoEx("shooter_light", -0.1)
    override fun initialize() { }
}

@Configurable
object Flywheel: Subsystem {
    private val flywheelMotor1: MotorEx = MotorEx("fwl").reversed().floatMode()
    private val flywheelMotor2: MotorEx = MotorEx("fwr").reversed().floatMode()
    private val flywheelMotors: MotorGroup = MotorGroup(flywheelMotor1, flywheelMotor2)
    private val voltageSensor: VoltageSensor by lazy { ActiveOpMode.hardwareMap.get(VoltageSensor::class.java, "Control Hub") }

    private val flywheelCoeffs: PSVCoeffs = PSVCoeffs(0.003, 0.09, 0.0004)
    @JvmField
    var IDLE_VELOCITY = 1500.0
    @JvmField
    var velocityGain = 1.01
    
    var targetVelocity: Double = 0.0
    val currentVelocity: Double get() = flywheelMotors.velocity


    fun calculatePow(): Double {
        val compensatedTarget = targetVelocity * velocityGain
        if (abs(compensatedTarget) < 1.0) return 0.0
        
        val error = compensatedTarget - currentVelocity
        
        // Use Feedforward + Proportional control for smooth adjustments near the target
        val ff = (flywheelCoeffs.kV * compensatedTarget) + (flywheelCoeffs.kS * sign(compensatedTarget))
        val p = flywheelCoeffs.kP * error
        
        return (ff + p).coerceIn(0.0, 1.0)
    }
    fun update() {
        val currVoltage: Double = voltageSensor.voltage
        // Normalize power output to 12V to compensate for voltage sag from other components
        //flywheelMotors.power = calculatePow() * (12.7 / currVoltage.coerceAtLeast(8.0))
        flywheelMotors.power = calculatePow()
    }
    fun reset() { targetVelocity = 0.0 }
    fun debug(): String = "Target Velocity = $targetVelocity \nCurrent Velocity = ${-flywheelMotors.velocity} \nCoeffs = $flywheelCoeffs \nPower = ${flywheelMotors.power}"
}

object Turret: Subsystem {
    private val turretServo1: ServoEx = ServoEx("lt")
    private val turretServo2: ServoEx = ServoEx("ft")

    private const val GEAR_RATIO: Double = 1.0
    private const val SERVO_RANGE: Double = 355.0

    var offset: Double = 0.0
        private set
    var targetAngle: Double = 0.0

    fun offset(by: Double) { offset += by }
    fun update() {
        turretServo1.position = (normalizeAngle300(targetAngle + offset) * (GEAR_RATIO / SERVO_RANGE)+0.500) //middle is 0.505
        turretServo2.position = (normalizeAngle300(targetAngle + offset) * (GEAR_RATIO / SERVO_RANGE)+0.510) //
    }
    fun reset() { offset = 0.0; targetAngle = 0.0 }
    fun debug(): String = "Target Angle = $targetAngle \nOffset = $offset \nCurrent Pos ition = ${turretServo1.position - 0.00130571*2.0}"

    override fun initialize() {
        val lt = ActiveOpMode.hardwareMap.get(com.qualcomm.robotcore.hardware.Servo::class.java, "lt")
        val ft = ActiveOpMode.hardwareMap.get(com.qualcomm.robotcore.hardware.Servo::class.java, "ft")
        (lt as? com.qualcomm.robotcore.hardware.PwmControl)?.pwmRange = com.qualcomm.robotcore.hardware.PwmControl.PwmRange(500.0, 2500.0)
        (ft as? com.qualcomm.robotcore.hardware.PwmControl)?.pwmRange = com.qualcomm.robotcore.hardware.PwmControl.PwmRange(500.0, 2500.0)
    }
}

internal fun normalizeAngle(angDeg: Double): Double {
    var a = angDeg % 360.0
    if (a <= -180.0) a += 360.0
    if (a > 180.0) a -= 360.0
    return a
}

internal fun normalizeAngle300(angDeg: Double): Double {
    var normalized = normalizeAngle(angDeg)
    return (normalized.coerceIn(-167.0,167.0))
}

enum class FlywheelState {
    PREDICTIVE_AUTO_AIM,
    AUTO_AIM,
    MANUAL
}

data class PSVCoeffs ( @JvmField var kP: Double, @JvmField var kS: Double, @JvmField var kV: Double)