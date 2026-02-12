@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.ServoGroup
import org.firstinspires.ftc.teamcode.Constants.ConfigConstants

object Turret: Subsystem {
    private val axonTurretServo: ServoEx = ServoEx(ConfigConstants.AXON_TURRET_SERVO,-0.1)
    private val torctexTurretServo: ServoEx = ServoEx(ConfigConstants.TORCTEX_TURRET_SERVO,-0.1)
    private val turretServos: ServoGroup = ServoGroup(axonTurretServo, torctexTurretServo)

    private const val GEAR_RATIO: Double = 0.9375
    private const val SERVO_RANGE: Double = 359.0

    internal var targetTurretAngle: Double = 0.0

    internal fun update() { turretServos.position = (normalizeAngle(targetTurretAngle) * (GEAR_RATIO / SERVO_RANGE) + 0.5) }
}

internal fun normalizeAngle(angDeg: Double): Double {
    var a = angDeg
    if (a < 0.0) { a += 360.0 }
    if (a > 180.0) { a -= 360.0 }
    return a
}
