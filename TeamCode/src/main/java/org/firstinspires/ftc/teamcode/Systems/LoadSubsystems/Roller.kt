package org.firstinspires.ftc.teamcode.Systems.LoadSubsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx

object Rollers: Subsystem {
    private val transferMotor: MotorEx = MotorEx("t")
    private val intakeMotor: MotorEx = MotorEx("i")
    private val shooterGateServo: ServoEx = ServoEx("sg")

    internal fun run(tPow: Double, iPow: Double) {
        transferMotor.power = -tPow
        intakeMotor.power = iPow
    }
    internal fun stop() = run(0.0,0.0)
    internal fun lockShooter() { shooterGateServo.position = 0.1 }
    internal fun unlockShooter() { shooterGateServo.position = 0.48 }
}