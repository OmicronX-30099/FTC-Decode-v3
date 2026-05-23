@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Systems.Load

import android.util.Log
import com.qualcomm.robotcore.hardware.DigitalChannel
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import kotlin.math.abs

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
    private var lastSideLightPosition: Double = Double.NaN

    override fun initialize() {
        lockShooter()
        lastSideLightPosition = Double.NaN
    }

    fun transfer(tPow: Double) { transferMotor.power = tPow }
    fun intake(iPow: Double) { intakeMotor.power = iPow }

    fun run(iPow: Double, tPow: Double) = transfer(tPow) .also { intake(iPow) }
    fun stop() = run(0.0,0.0)

    fun lockShooter() { shooterGateServo.position = 0.32 }
    fun unlockShooter() { shooterGateServo.position = 0.475 }

    fun update(ballCount: Int = BreakBeam.refreshBallCount()) {
        val sidePos = when (ballCount) {
            3 -> 0.5
            2 -> 0.388
            1 -> 0.277
            else -> 0.0
        }
        if (lastSideLightPosition.isNaN() || abs(sidePos - lastSideLightPosition) > 0.001) {
            sideLight1.position = sidePos
            sideLight2.position = sidePos
            lastSideLightPosition = sidePos
        }

        if (isFeeding) return

        if (ballCount == 3) {
            if (fullStartTime == -1L) {
                ActiveOpMode.gamepad1.rumble(200)
                fullStartTime = System.currentTimeMillis()
            }
        }

        if (fullStartTime != -1L) {
            if (System.currentTimeMillis() - fullStartTime > 10) {
                stop()
                if (ballCount < 3) {
                    fullStartTime = -1L
                }
            }
        } else {
            if (ballCount >= 1) {
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
    val pos2Occupied: Boolean get() = !bb3.state || !bb4.state
    val pos3Occupied: Boolean get() = !bb5.state || !bb6.state

    private var oneStartTime: Long = -1L
    private var twoStartTime: Long = -1L
    private var threeStartTime: Long = -1L
    private var lastRefreshTime: Long = 0L
    var cachedBallCount: Int = 0
        private set
    val ballCount: Int get() = refreshBallCount()

    fun refreshBallCount(minIntervalMs: Long = 0L): Int {
        val now = System.currentTimeMillis()
        if (minIntervalMs > 0L && now - lastRefreshTime < minIntervalMs) {
            return cachedBallCount
        }

        val p1 = pos1Occupied
        val p2 = pos2Occupied
        val p3 = pos3Occupied

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

        cachedBallCount = when {
            threeStartTime != -1L && now - threeStartTime > 25 -> 3
            twoStartTime != -1L && now - twoStartTime > 25 -> 2
            oneStartTime != -1L && now - oneStartTime > 25 -> 1
            else -> 0
        }
        lastRefreshTime = now
        return cachedBallCount
    }

    val isFull: Boolean get() = refreshBallCount() == 3

    override fun initialize() {
        oneStartTime = -1L
        twoStartTime = -1L
        threeStartTime = -1L
        lastRefreshTime = 0L
        cachedBallCount = 0
    }
}

object Stupid {  }
