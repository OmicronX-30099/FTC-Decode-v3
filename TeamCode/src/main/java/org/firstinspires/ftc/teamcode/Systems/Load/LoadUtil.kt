@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Systems.Load

import com.qualcomm.robotcore.hardware.DigitalChannel
import dev.nextftc.bindings.button
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage

object Rollers: Subsystem {
    private val transferMotor: MotorEx = MotorEx("t")
    private val intakeMotor: MotorEx = MotorEx("i")
    private val shooterGateServo: ServoEx = ServoEx("sg",-0.1)

    override fun initialize() { lockShooter() }

    fun transfer(tPow: Double) { transferMotor.power = tPow }
    fun intake(iPow: Double) { intakeMotor.power = iPow }

    fun run(iPow: Double, tPow: Double) = transfer(tPow) .also { intake(iPow) }
    fun stop() = run(0.0,0.0)

    fun lockShooter() { shooterGateServo.position = 0.5 }
    fun unlockShooter() { shooterGateServo.position = 0.2 }
}

object BilinearIndexMachine: Subsystem {
    private val leftModuleServo: ServoEx = ServoEx("lm",-0.1)
    private val rightModuleServo: ServoEx = ServoEx("rm",-0.1)
    private val transferGate: ServoEx = ServoEx("transfer_gate",-0.1)

    fun unlockTransfer() { transferGate.position = 0.3 }
    fun lockTransfer() { transferGate.position = 0.6 }

    fun transferLeft() {leftModuleServo.position = 0.95 .also { rightModuleServo.position = 0.05 } }
    fun transferRight() {leftModuleServo.position = 0.05 .also { rightModuleServo.position = 0.95 } }
    fun transferMiddle() {leftModuleServo.position = 0.95 .also { rightModuleServo.position = 0.05 } }
    override fun initialize() { transferMiddle(); unlockTransfer() }
}

object BreakBeam: Subsystem {
    val bb: DigitalChannel by lazy { ActiveOpMode.hardwareMap.get(DigitalChannel::class.java, "bb") }
    val bbTrigger = button { !bb.state }

    var count = 0.0
    var ballEntered: Boolean = true

    override fun initialize() {
        if (ROBOT.currStage == Stage.TELEOP) { return }
        bbTrigger
            .whenBecomesTrue {
                if (ballEntered) {
                    ballEntered = false
                    SequentialGroup(
                        Delay(0.17),
                        instant { ballEntered = true; count++ }
                    ).apply {
                        schedule()
                    }
                }
            }
    }
}