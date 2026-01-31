package org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx

object ShooterGateSubsystem: Subsystem {
    private val shooterGateServo: ServoEx = ServoEx("sg", -0.1)

    override fun initialize() {
        open()
        block()
    }

    internal fun open() { shooterGateServo.position = 0.1 }
    internal fun block() { shooterGateServo.position = 0.45 }
}