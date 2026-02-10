package org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems

import dev.nextftc.control2.feedback.PIDController
import dev.nextftc.control2.feedforward.SimpleFFCoefficients
import dev.nextftc.control2.feedforward.SimpleFeedforward
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.Constants.ConfigConstants

object Flywheel: Subsystem {
    private val topFlywheelMotor: MotorEx = MotorEx(ConfigConstants.TOP_FLYWHEEL_MOTOR)
    private val bottomFlywheelMotor: MotorEx = MotorEx(ConfigConstants.BOTTOM_FLYWHEEL_MOTOR)
    private val flywheelMotors: MotorGroup = MotorGroup(topFlywheelMotor, bottomFlywheelMotor)

    private val flywheelPIDController: PIDController = PIDController(0.0075,0.0,0.0)

    private val flywheelFFCoefficients: SimpleFFCoefficients = SimpleFFCoefficients(0.064,0.00043,0.0)
    private val flywheelFFController: SimpleFeedforward = SimpleFeedforward(flywheelFFCoefficients)

    internal const val IDLE_VELOCITY: Double = 1140.0

    internal var flywheelTarget: Double = 0.0

    internal fun update() { flywheelMotors.power = flywheelPIDController.calculate(error = flywheelTarget - flywheelMotors.velocity) + flywheelFFController.calculate(flywheelTarget) }
}

internal enum class FlywheelState {
    AUTO_AIM,
    IDLE;
}