package org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx

object BIMSubsystem: Subsystem {
    private val leftModuleServo: ServoEx = ServoEx("lm",-0.1)
    private val rightModuleServo: ServoEx = ServoEx("rm",-0.1)

    internal fun loadLeft() {
        leftModuleServo.position = 1.0
        rightModuleServo.position = 0.0
    }
    internal fun loadRight() {
        leftModuleServo.position = 0.0
        rightModuleServo.position = 1.0
    }
    internal fun loadMiddle() {
        leftModuleServo.position = 0.0
        rightModuleServo.position = 0.0
    }
}