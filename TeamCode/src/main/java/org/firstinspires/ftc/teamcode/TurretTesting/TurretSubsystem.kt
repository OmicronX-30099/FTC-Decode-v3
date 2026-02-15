package org.firstinspires.ftc.teamcode.TurretTesting

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx

object TurretSubsystem: Subsystem {
    val axonServo: ServoEx = ServoEx("lt")
    val torctexServo: ServoEx = ServoEx("ft")

    var targetAngle: Double = 0.0

    fun update() {  }
}