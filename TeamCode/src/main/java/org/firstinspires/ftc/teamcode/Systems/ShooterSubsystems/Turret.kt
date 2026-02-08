package org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.ServoGroup
import org.firstinspires.ftc.teamcode.Constants.ConfigConstants

object Turret: Subsystem {
    private val axonTurretServo: ServoEx = ServoEx(ConfigConstants.axonTurretServo,-0.1)
    private val torctexTurretServo: ServoEx = ServoEx(ConfigConstants.torctexTurretServo,-0.1)
    private val turretServos: ServoGroup = ServoGroup(axonTurretServo, torctexTurretServo)

    private const val GEAR_RATIO: Double = 15.0 / 16.0

    internal var targetTurretAngle: Double = 0.0

    internal fun update() { turretServos.position = (normalizeAngle(targetTurretAngle) * (GEAR_RATIO / 355.0) + 0.5) }
}

internal enum class TurretState {
    AUTO_AIM,
    MANUAL;
}

internal fun normalizeAngle(angDeg: Double): Double {
    var a = angDeg
    if (a < 0.0) { a += 360.0 }
    if (a > 180.0) { a -= 360.0 }
    return a
}