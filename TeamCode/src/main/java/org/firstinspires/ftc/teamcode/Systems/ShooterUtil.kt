package org.firstinspires.ftc.teamcode.Systems

import dev.nextftc.control2.filters.KalmanFilter
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.ServoGroup

object Turret: Subsystem {
    private val turretServo1: ServoEx = ServoEx("")
    private val turretServo2: ServoEx = ServoEx("")
    private val turretServos: ServoGroup = ServoGroup(turretServo1, turretServo2)

    private const val GEAR_RATIO = 0.9375
    private const val SERVO_RANGE = 358.75

    var targetAngle: Double = 0.0

    fun updateTarget() { turretServos.position = ( ( normalizeAngle(targetAngle) * ( GEAR_RATIO / SERVO_RANGE ) ) + 0.5 ) }
}

object Flywheel: Subsystem {
    private val flywheelMotor1: MotorEx = MotorEx("")
    private val flywheelMotor2: MotorEx = MotorEx("")
    private val flywheelMotor: MotorGroup = MotorGroup(flywheelMotor1, flywheelMotor2)

    private val x = KalmanFilter()
}

fun normalizeAngle(ang: Double): Double =
    if (ang < -180.0) { ang + 360.0 }
    else if (ang > 180.0) { ang - 360.0 }
    else { ang }