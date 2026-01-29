package org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.ServoGroup
import org.firstinspires.ftc.teamcode.Final.Subsystems.State

object TurretSubsystem: Subsystem {
    private val axonTurretServo: ServoEx = ServoEx("lt",-0.1)
    private val torctexTurretServo: ServoEx = ServoEx("ft",-0.1)
    private val turretServos: ServoGroup = ServoGroup(axonTurretServo, torctexTurretServo)

    private const val GEAR_RATIO: Double = 0.9375
    internal var targetAngle: Double = 0.0

    internal fun updateTurret() { turretServos.position = ((normalizeAngle(targetAngle) * GEAR_RATIO) / 355.0) + 0.5 }
}

internal enum class TurretState: State {
    AUTO_AIM,
    MANUAL;
    override val angVelScalar: Double = 0.0
    override val linVelScalar: Double = 0.0
}

internal fun normalizeAngle(angDeg: Double): Double {
    var input = angDeg
    if (input < 0.0) { input += 360.0 }
    if (input > 180.0) { input -= 360.0 }
    return input
}