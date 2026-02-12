@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems

import dev.nextftc.control2.feedback.PIDController
import dev.nextftc.control2.feedforward.SimpleFFCoefficients
import dev.nextftc.control2.feedforward.SimpleFeedforward
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.Constants.ConfigConstants
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.ActiveOpMode.hardwareMap


object Flywheel: Subsystem {
    private val topFlywheelMotor: MotorEx = MotorEx(ConfigConstants.TOP_FLYWHEEL_MOTOR)
    private val bottomFlywheelMotor: MotorEx = MotorEx(ConfigConstants.BOTTOM_FLYWHEEL_MOTOR)
    private val flywheelMotors: MotorGroup = MotorGroup(topFlywheelMotor, bottomFlywheelMotor)
    private val battery: VoltageSensor by lazy { ActiveOpMode.hardwareMap.get(VoltageSensor::class.java, "Control Hub") }

    private val flywheelPIDController: PIDController = PIDController(0.0075,0.0,0.0)

    private val flywheelFFCoefficients: SimpleFFCoefficients = SimpleFFCoefficients(0.064,0.00043,0.0)
    private val flywheelFFController: SimpleFeedforward = SimpleFeedforward(flywheelFFCoefficients)

    private const val V_NOMINAL = 12.0
    internal const val IDLE_VELOCITY: Double = 1140.0
    
    internal var flywheelTarget: Double = 0.0

    internal fun isAtTarget(): Boolean { return ((flywheelTarget - 20.0) < flywheelMotors.velocity) && ((flywheelTarget + 40.0) > flywheelMotors.velocity) }
    
    internal fun update() {
        val pid = flywheelPIDController.calculate(error = flywheelTarget - flywheelMotors.velocity)
        val ff  = flywheelFFController.calculate(flywheelTarget)
        val volt = battery.voltage ?: V_NOMINAL.coerceAtleast(8.0)
        val pow = ((pid + ff) * (V_NOMINAL / volt)).coerceIn(-1.0,1.0)

        flywheelMotors.power = pow
        ActiveOpMode.telemetry.addData("flywheel power:", pow)
    }
}

internal enum class FlywheelState {
    AUTO_AIM,
    IDLE,
    STOPPED;
}
