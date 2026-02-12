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

    private val flywheelPIDController: PIDController = PIDController(0.0075,0.0,0.0)

    private val flywheelFFCoefficients: SimpleFFCoefficients = SimpleFFCoefficients(0.064,0.00043,0.0)
    private val flywheelFFController: SimpleFeedforward = SimpleFeedforward(flywheelFFCoefficients)

    internal const val IDLE_VELOCITY: Double = 1140.0

    internal var flywheelTarget: Double = 0.0

    private const val VNOM = 12.0
    private var voltageSensor: VoltageSensor? = null

    override fun initialize() {
        // other subsystem init...
        init(hardwareMap)   // <- add this
    }


    fun init(hwMap: HardwareMap) {
        // Use the first sensor (or you can pick min valid if you want)
        voltageSensor = hwMap.voltageSensor.firstOrNull()
    }

    private fun batteryVoltage(): Double {
        val v = voltageSensor?.voltage ?: VNOM
        return if (v > 0.0) v else VNOM
    }

    internal fun update() {
        val pid = flywheelPIDController.calculate(error = flywheelTarget - flywheelMotors.velocity)
        val ff  = flywheelFFController.calculate(flywheelTarget)

        val u = pid + ff
        val v = batteryVoltage().coerceAtLeast(8.0)     // safety clamp
        val uComp = (u * (VNOM / v)).coerceIn(-1.0, 1.0)

        flywheelMotors.power = uComp

        ActiveOpMode.telemetry.addData("flywheel power:", uComp)
        }
}

internal enum class FlywheelState {
    AUTO_AIM,
    IDLE;
}