package org.firstinspires.ftc.teamcode.Systems.LoadSubsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx

object Rollers: Subsystem {
    private val intakeMotor: MotorEx = MotorEx("")
    private val transferMotor: MotorEx = MotorEx("")

    fun intake(pow: Double) { intakeMotor.power = pow }
    fun transfer(pow: Double) { transferMotor.power = -pow }
    fun run(iPow: Double, tPow: Double) =
            intake(iPow)
    .also { transfer(tPow) }
    fun stop() = run(0.0,0.0)
}

object BIM: Subsystem {
    private val leftBIMServo: ServoEx = ServoEx("")
    private val rightBIMServo: ServoEx = ServoEx("")
    private val BIMGate: ServoEx = ServoEx("")

    fun unlockTransfer() { BIMGate.position = 0.3 }
    fun lockTransfer() { BIMGate.position = 0.6 }

    fun toLeft() { leftBIMServo.position = 0.0; rightBIMServo.position = 1.0 }
    fun toRight() { leftBIMServo.position = 1.0; rightBIMServo.position = 0.0 }
    fun split() { leftBIMServo.position = 0.0; rightBIMServo.position = 0.0 }

    override fun initialize() { split(); unlockTransfer() }
}