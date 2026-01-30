package org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems

import dev.nextftc.control2.feedback.PIDCoefficients
import dev.nextftc.control2.feedback.PIDController
import dev.nextftc.control2.feedforward.SimpleFFCoefficients
import dev.nextftc.control2.feedforward.SimpleFeedforward
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.Systems.State

object FlywheelSubsystem: Subsystem {
    private val topFlywheelMotor: MotorEx = MotorEx("fwt")
    private val bottomFlywheelMotor: MotorEx = MotorEx("fwb")
    private val flywheelMotors: MotorGroup = MotorGroup(topFlywheelMotor, bottomFlywheelMotor)

    private val flywheelPIDCoefficients: PIDCoefficients = PIDCoefficients(0.0075,0.0,0.0)
    private val flywheelFFCoefficients: SimpleFFCoefficients = SimpleFFCoefficients(0.064,0.00043,0.0)

    private val flywheelPIDController: PIDController = PIDController(flywheelPIDCoefficients)
    private val flywheelFFController: SimpleFeedforward = SimpleFeedforward(flywheelFFCoefficients)

    internal var flywheelTargetVel: Double = 0.0

    internal fun updateFlywheel() {
        val currVel: Double = flywheelMotors.velocity
        val pow: Double = flywheelPIDController.calculate(error = flywheelTargetVel - currVel, errorDerivative = null) + flywheelFFController.calculate(flywheelTargetVel)
        flywheelMotors.power = pow
    }
}

internal enum class FlywheelState: State {
    AUTO_AIM,
    MANUAL;
    override val angVelScalar: Double = 0.0
    override val linVelScalar: Double = 0.0
}