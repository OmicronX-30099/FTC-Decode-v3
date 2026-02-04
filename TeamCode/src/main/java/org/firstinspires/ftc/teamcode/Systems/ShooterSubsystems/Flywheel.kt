package org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems

import dev.nextftc.control2.feedback.PIDController
import dev.nextftc.control2.feedforward.SimpleFFCoefficients
import dev.nextftc.control2.feedforward.SimpleFeedforward
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.MotorEx

// Flywheel Object
object Flywheel: Subsystem {
    // Hardware
    private val topFlywheelMotor: MotorEx = MotorEx("fwt")
    private val bottomFlywheelMotor: MotorEx = MotorEx("fwb")
    private val flywheelMotors: MotorGroup = MotorGroup(topFlywheelMotor, bottomFlywheelMotor)

    // PID Controller
    private val flywheelPIDController: PIDController = PIDController(0.0075,0.0,0.0)

    // FF Coeffs and Controller
    private val flywheelFFCoefficients: SimpleFFCoefficients = SimpleFFCoefficients(0.064,0.00043,0.0)
    private val flywheelFFController: SimpleFeedforward = SimpleFeedforward(flywheelFFCoefficients)

    // Velocity that the flywheel should idle at
    internal const val IDLE_VELOCITY: Double = 1140.0

    // Target flywheel velocity(TPS)
    internal var flywheelTarget: Double = 0.0

    // Function to set calculated power to motors
    internal fun update() { flywheelMotors.power = flywheelPIDController.calculate(error = flywheelTarget - flywheelMotors.velocity) + flywheelFFController.calculate(flywheelTarget) }
}

// Enum to track state of Flywheel
// AUTO_AIM: Uses distance from goal and equation to calculate flywheel velocity
// IDLE: Flywheel idles at a certain velocity until switched to AUTO_AIM, also acts as OFF state
internal enum class FlywheelState {
    AUTO_AIM,
    IDLE;
}