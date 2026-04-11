@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Systems.Load

import android.util.Log
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
    init {
        Log.d("Rollers", "Initializing")
    }
    private val transferMotor: MotorEx = MotorEx("t").reversed()
    private val intakeMotor: MotorEx = MotorEx("i")
    private val shooterGateServo: ServoEx = ServoEx("shooter_gate",-0.1)
    private val sideLight1: ServoEx = ServoEx("left_light", -0.1)
    private val sideLight2: ServoEx = ServoEx("right_light", -0.1)

    var isFeeding: Boolean = false
    private var wasFull: Boolean = false

    override fun initialize() { lockShooter() }

    fun transfer(tPow: Double) { transferMotor.power = tPow }
    fun intake(iPow: Double) { intakeMotor.power = iPow }

    fun run(iPow: Double, tPow: Double) = transfer(tPow) .also { intake(iPow) }
    fun stop() = run(0.0,0.0)

    fun lockShooter() { shooterGateServo.position = 0.5 }
    fun unlockShooter() { shooterGateServo.position = 0.2 }

    fun update() {
        val sidePos = when (BreakBeam.ballCount) {
            3 -> 0.5
            2 -> 0.388
            1 -> 0.277
            else -> 0.0
        }
        sideLight1.position = sidePos
        sideLight2.position = sidePos

        if (isFeeding) return

        if (BreakBeam.isFull) {
            if (!wasFull) {
                ActiveOpMode.gamepad1.rumble(500)
                wasFull = true
            }
            stop()
        } else {
            wasFull = false
            if (BreakBeam.ballCount >= 1) {
                transfer(0.0)
            }
        }
    }
}

object BreakBeam: Subsystem {
    private val bb1 by lazy { ActiveOpMode.hardwareMap.get(DigitalChannel::class.java, "bb1") }
    private val bb2 by lazy { ActiveOpMode.hardwareMap.get(DigitalChannel::class.java, "bb2") }
    private val bb3 by lazy { ActiveOpMode.hardwareMap.get(DigitalChannel::class.java, "bb3") }
    private val bb4 by lazy { ActiveOpMode.hardwareMap.get(DigitalChannel::class.java, "bb4") }
    private val bb5 by lazy { ActiveOpMode.hardwareMap.get(DigitalChannel::class.java, "bb5") }
    private val bb6 by lazy { ActiveOpMode.hardwareMap.get(DigitalChannel::class.java, "bb6") }

    val pos1Occupied: Boolean get() = !bb1.state || !bb2.state
    val pos2Occupied: Boolean get() = !bb3.state || !bb4.state
    val pos3Occupied: Boolean get() = !bb5.state || !bb6.state

    val ballCount: Int get() = (if (pos1Occupied) 1 else 0) + (if (pos2Occupied) 1 else 0) + (if (pos3Occupied) 1 else 0)
    val isFull: Boolean get() = ballCount == 3

    override fun initialize() { }
}

object Stupid {  }