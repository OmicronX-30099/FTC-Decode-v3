package org.firstinspires.ftc.teamcode.Systems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.ServoGroup

object Flywheel: Subsystem {

}

object Turret: Subsystem {
    private val axonTurretServo: ServoEx = ServoEx("")
    private val torctexTurretServo: ServoEx = ServoEx("")
    private val turretServos: ServoGroup = ServoGroup(axonTurretServo,torctexTurretServo)

    private const val GEAR_RATIO = 0.9375
    private const val SERVO_RANGE = 359.0

    var targetTurretAng: Double = 0.0

    fun update() { turretServos.position = ((normalizeAngle(targetTurretAng) * (GEAR_RATIO / SERVO_RANGE)) + 0.5) }
}

internal fun normalizeAngle(ang: Double): Double {
    var _ang = ang
    if (_ang < 0.0) { _ang += 360.0 }
    if (_ang > 180.0) { _ang -= 360.0 }
    return _ang
}