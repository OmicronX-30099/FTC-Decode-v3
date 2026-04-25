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
    private val intakeMotor: MotorEx = MotorEx("i").reversed()
    private val shooterGateServo: ServoEx = ServoEx("shooter_gate",-0.1)
    private val sideLight1: ServoEx = ServoEx("left_light", -0.1)
    private val sideLight2: ServoEx = ServoEx("right_light", -0.1)

    var isFeeding: Boolean = false
    private var fullStartTime: Long = -1

    override fun initialize() { lockShooter() }

    fun transfer(tPow: Double) { transferMotor.power = tPow }
    fun intake(iPow: Double) { intakeMotor.power = iPow }

    fun run(iPow: Double, tPow: Double) = transfer(tPow) .also { intake(iPow) }
    fun stop() = run(0.0,0.0)

    fun lockShooter() { shooterGateServo.position = 0.32 }
    fun unlockShooter() { shooterGateServo.position = 0.475 }

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
            if (fullStartTime == -1L) {
                ActiveOpMode.gamepad1.rumble(200)
                fullStartTime = System.currentTimeMillis()
            }
        }

        if (fullStartTime != -1L) {
            if (System.currentTimeMillis() - fullStartTime > 10) {
                stop()
                if (!BreakBeam.isFull) {
                    fullStartTime = -1L
                }
            }
        } else {
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
    //map of bb: intake to shooter: bb1,bb2 -> bb4,bb3 -> bb6,bb5
    val pos1Occupied: Boolean get() = !bb1.state || !bb2.state //closest to intake, bb1 broken
    val pos2Occupied: Boolean get() = !bb3.state && !bb4.state
    val pos3Occupied: Boolean get() = !bb5.state || !bb6.state

    private var oneStartTime: Long = -1L
    private var twoStartTime: Long = -1L
    private var threeStartTime: Long = -1L

    val ballCount: Int
        get() {
            val p1 = pos1Occupied
            val p2 = pos2Occupied
            val p3 = pos3Occupied
            val now = System.currentTimeMillis()

            if (p3) {
                if (oneStartTime == -1L) oneStartTime = now
            } else {
                oneStartTime = -1L
            }

            if (p2 && p3) {
                if (twoStartTime == -1L) twoStartTime = now
            } else {
                twoStartTime = -1L
            }

            if (p1 && p2 && p3) {
                if (threeStartTime == -1L) threeStartTime = now
            } else {
                threeStartTime = -1L
            }

            return when {
                threeStartTime != -1L && now - threeStartTime > 150 -> 3
                twoStartTime != -1L && now - twoStartTime > 150 -> 2
                oneStartTime != -1L && now - oneStartTime > 150 -> 1
                else -> 0
            }
        }

    val isFull: Boolean get() = ballCount == 3

    override fun initialize() {
        oneStartTime = -1L
        twoStartTime = -1L
        threeStartTime = -1L
    }
}

object Stupid {  }
