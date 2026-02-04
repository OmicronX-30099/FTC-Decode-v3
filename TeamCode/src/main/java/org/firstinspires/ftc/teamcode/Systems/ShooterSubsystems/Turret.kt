package org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.ServoGroup

// Turret Object
object Turret: Subsystem {
    // Hardware
    private val axonTurretServo: ServoEx = ServoEx("lt",-0.1)
    private val torctexTurretServo: ServoEx = ServoEx("ft",-0.1)
    private val turretServos: ServoGroup = ServoGroup(axonTurretServo, torctexTurretServo)

    // Turret Gear ratio
    private const val GEAR_RATIO: Double = 15.0 / 16.0

    // Target angle for turret to turn to
    internal var targetTurretAngle: Double = 0.0

    // Function to calculate servo position and set to servos
    internal fun update() { turretServos.position = (normalizeAngle(targetTurretAngle) * (GEAR_RATIO / 355.0) + 0.5) }
}

// Enum to track state of turret
// AUTO_AIM: Uses current position to automatically calculate target turret angle
// MANUAL: For manual control of the turret using gamepad, also acts as OFF state
internal enum class TurretState {
    AUTO_AIM,
    MANUAL;
}

// Function to put an angle within range (-180,180]
internal fun normalizeAngle(angDeg: Double): Double {
    var a = angDeg
    if (a < 0.0) { a += 360.0 }
    if (a > 180.0) { a -= 360.0 }
    return a
}
