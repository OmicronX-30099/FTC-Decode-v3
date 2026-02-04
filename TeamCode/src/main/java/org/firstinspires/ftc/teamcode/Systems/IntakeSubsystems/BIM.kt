package org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx

object BIM: Subsystem {
    private val leftModuleServo: ServoEx = ServoEx("lm",-0.1)
    private val rightModuleServo: ServoEx = ServoEx("rm",-0.1)
    private val transferGate: ServoEx = ServoEx("transfer_gate",-0.1)

    internal fun unlockTransfer() { transferGate.position = 0.7 }
    internal fun lockTransfer() { transferGate.position = 0.96 }

    internal fun transferLeft() {leftModuleServo.position = 1.0 .also { rightModuleServo.position = 0.0 } }
    internal fun transferRight() {leftModuleServo.position = 0.0 .also { rightModuleServo.position = 1.0 } }
    internal fun transferMiddle() {leftModuleServo.position = 0.0 .also { rightModuleServo.position = 0.0 } }
    override fun initialize() {
        transferMiddle()
        lockTransfer()
    }
}