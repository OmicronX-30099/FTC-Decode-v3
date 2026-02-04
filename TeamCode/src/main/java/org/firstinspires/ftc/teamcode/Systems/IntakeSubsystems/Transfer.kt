package org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx

object Transfer: Subsystem {
    private val intakeMotor: MotorEx = MotorEx("i")
    private val transferMotor: MotorEx = MotorEx("t")
    private val shooterGate: ServoEx = ServoEx("sg")

    internal fun intake(power: Double) { intakeMotor.power = power }
    internal fun transfer(power: Double) { transferMotor.power = -power }

    internal fun unlockShooter() { shooterGate.position = 0.1 }
    internal fun lockShooter() { shooterGate.position = 0.48 }
}