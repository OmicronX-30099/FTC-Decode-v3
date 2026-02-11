@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Systems.LoadSubsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.Constants.ConfigConstants

object Rollers: Subsystem {
    private val transferMotor: MotorEx = MotorEx(ConfigConstants.TRANSFER_MOTOR)
    private val intakeMotor: MotorEx = MotorEx(ConfigConstants.INTAKE_MOTOR)
    private val shooterGateServo: ServoEx = ServoEx(ConfigConstants.SHOOTER_GATE_SERVO,-0.1)

    override fun initialize() { lockShooter() }

    internal fun run(tPow: Double, iPow: Double) {
        transferMotor.power = -tPow
        intakeMotor.power = iPow
    }
    internal fun stop() = run(0.0,0.0)
    internal fun lockShooter() { shooterGateServo.position = 0.5 }
    internal fun unlockShooter() { shooterGateServo.position = 0.2 }
}