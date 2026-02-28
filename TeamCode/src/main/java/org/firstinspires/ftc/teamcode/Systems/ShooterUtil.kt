package org.firstinspires.ftc.teamcode.Systems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.ServoGroup

object Turret: Subsystem {
    private val turretServo1: ServoEx = ServoEx("")
    private val turretServo2: ServoEx = ServoEx("")
    private val turretServos: ServoGroup = ServoGroup(turretServo1, turretServo2)

    private const val GEAR_RATIO: Double = 0.9375
    private const val SERVO_RANGE: Double = 358.75

    private var turretOffset: Double = 0.0
    var targetTurretAngle: Double = 0.0

    fun offset(by: Double) { turretOffset += by }
    fun update() { turretServos.position = (normalizeAngle(targetTurretAngle + turretOffset) * (GEAR_RATIO / SERVO_RANGE) + 0.5) }
}

internal fun normalizeAngle(angDeg: Double): Double {
    var a = angDeg
    while (a < 0.0) { a += 360.0 }
    while (a > 180.0) { a -= 360.0 }
    return a
}