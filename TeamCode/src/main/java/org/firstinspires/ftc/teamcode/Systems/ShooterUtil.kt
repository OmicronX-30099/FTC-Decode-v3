package org.firstinspires.ftc.teamcode.Systems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.ServoGroup

object Turret: Subsystem {
    private val turretServo1 = ServoEx("")
    private val turretServo2 = ServoEx("")
    private val turretServos = ServoGroup(turretServo1, turretServo2)

    var turret
}