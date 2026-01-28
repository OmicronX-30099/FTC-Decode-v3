package org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx

object IntakeSubsystem: Subsystem {
    private val intakeMotor: MotorEx = MotorEx("i")

    internal fun intake(pow: Double) { intakeMotor.power = pow }
}