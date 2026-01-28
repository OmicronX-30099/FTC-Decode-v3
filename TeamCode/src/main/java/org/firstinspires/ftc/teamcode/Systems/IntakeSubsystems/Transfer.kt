package org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx

object TransferSubsystem: Subsystem {
    private val transferMotor: MotorEx = MotorEx("t")

    internal fun transfer(pow: Double) { transferMotor.power = -pow }
}