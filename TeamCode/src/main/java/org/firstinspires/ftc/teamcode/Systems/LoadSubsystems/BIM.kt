package org.firstinspires.ftc.teamcode.Systems.LoadSubsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.Constants.ConfigConstants

object BilinearIndexMachine: Subsystem {
    private val leftModuleServo: ServoEx = ServoEx(ConfigConstants.leftModuleServo,-0.1)
    private val rightModuleServo: ServoEx = ServoEx(ConfigConstants.rightModuleServo,-0.1)
    private val transferGate: ServoEx = ServoEx(ConfigConstants.transferGateServo,-0.1)

    internal fun unlockTransfer() { transferGate.position = 0.3 }
    internal fun lockTransfer() { transferGate.position = 0.6 }

    internal fun transferLeft() {leftModuleServo.position = 1.0 .also { rightModuleServo.position = 0.0 } }
    internal fun transferRight() {leftModuleServo.position = 0.0 .also { rightModuleServo.position = 1.0 } }
    internal fun transferMiddle() {leftModuleServo.position = 0.0 .also { rightModuleServo.position = 0.0 } }
    override fun initialize() {
        transferMiddle()
        lockTransfer()
    }
}