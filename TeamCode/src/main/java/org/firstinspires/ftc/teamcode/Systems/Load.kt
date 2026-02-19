package org.firstinspires.ftc.teamcode.Systems

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.subsystems.SubsystemGroup
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.Util.and
import org.firstinspires.ftc.teamcode.Util.exec
import org.firstinspires.ftc.teamcode.Util.execute

object Load: SubsystemGroup(Rollers, BIM) {
    fun shootCommand(iPow: Double, tPow: Double, delay: Double): Command
        = SequentialGroup(
            InstantCommand {
                BIM exec {
                    unlockTransfer()
                    split()
                }
                Rollers exec {
                    run(iPow, tPow)
                    unlockShooter()
                }
            },
            Delay(delay),
            InstantCommand {
                Rollers execute {
                    stop()
                    lockShooter()
                }
            }
        )
}

object Rollers: Subsystem {
    private val intakeMotor: MotorEx = MotorEx("")
    private val transferMotor: MotorEx = MotorEx("")
    private val shooterGateServo: ServoEx = ServoEx("")

    fun intake(pow: Double) { intakeMotor.power = pow }
    fun transfer(pow: Double) { transferMotor.power = -pow }

    fun run(iPow: Double, tPow: Double) { intake(iPow) and { transfer(tPow) } }
    fun stop() = run(0.0,0.0)

    fun unlockShooter() { shooterGateServo.position = 0.2 }
    fun lockShooter() { shooterGateServo.position = 0.5 }

    override fun initialize() { lockShooter() }
}

object BIM: Subsystem {
    private val leftBIMServo: ServoEx = ServoEx("")
    private val rightBIMServo: ServoEx = ServoEx("")
    private val BIMGate: ServoEx = ServoEx("")

    fun unlockTransfer() { BIMGate.position = 0.3 }
    fun lockTransfer() { BIMGate.position = 0.6 }

    fun toLeft() { leftBIMServo.position = 0.0 and {rightBIMServo.position = 1.0 } }
    fun toRight() { leftBIMServo.position = 1.0 and { rightBIMServo.position = 0.0 } }
    fun split() { leftBIMServo.position = 0.0 and { rightBIMServo.position = 0.0 } }

    override fun initialize() { split() and { unlockTransfer() } }
}