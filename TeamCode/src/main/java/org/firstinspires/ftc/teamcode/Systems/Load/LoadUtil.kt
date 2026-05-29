@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Systems.Load

import android.util.Log
import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DigitalChannel
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage
import kotlin.math.abs

private const val FULL_CONFIRM_MS = 160L
private const val POWER_EPSILON = 0.001

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
    var intakePower: Double = 0.0
        private set
    var transferPower: Double = 0.0
        private set
    private var fullStartTime: Long = -1
    private var fullRumbleSent: Boolean = false
    private var lastSideLightPosition: Double = Double.NaN

    override fun initialize() {
        lockShooter()
        intakePower = 0.0
        transferPower = 0.0
        fullStartTime = -1L
        fullRumbleSent = false
        lastSideLightPosition = Double.NaN
    }

    fun transfer(tPow: Double) {
        if (abs(transferPower - tPow) <= POWER_EPSILON) return
        transferPower = tPow
        transferMotor.power = tPow
    }
    fun intake(iPow: Double) {
        if (abs(intakePower - iPow) <= POWER_EPSILON) return
        intakePower = iPow
        intakeMotor.power = iPow
    }

    fun run(iPow: Double, tPow: Double) {
        if (iPow < -POWER_EPSILON || tPow < -POWER_EPSILON) {
            BreakBeam.releaseFullHold()
            fullStartTime = -1L
            fullRumbleSent = false
        }
        transfer(tPow)
        intake(iPow)
    }
    fun stop() = run(0.0,0.0)

    fun lockShooter() { shooterGateServo.position = 0.32 }
    fun unlockShooter() { shooterGateServo.position = 0.475 }

    fun update(ballCount: Int = BreakBeam.refreshBallCount()) {
        if (isFeeding) {
            fullStartTime = -1L
            fullRumbleSent = false
            return
        }

        if (BreakBeam.isFullHeld) {
            fullStartTime = -1L
            if (ROBOT.currStage != Stage.TELEOP) stop()
            return
        }

        if (ballCount >= 3) {
            val now = System.currentTimeMillis()
            if (fullStartTime == -1L) {
                fullStartTime = now
            }
            if (now - fullStartTime >= FULL_CONFIRM_MS) {
                if (ROBOT.currStage == Stage.TELEOP) {
                    if (!fullRumbleSent) {
                        ActiveOpMode.gamepad1.rumble(200)
                        fullRumbleSent = true
                    }
                } else {
                    stop()
                    BreakBeam.holdFull()
                }
                fullStartTime = -1L
            }
            return
        }

        fullStartTime = -1L
        if (ballCount >= 1) {
            transfer(0.0)
        }
    }
}

@Configurable
object BreakBeam: Subsystem {
    @JvmField var BB1_ENABLED: Boolean = false
    @JvmField var OCCUPANCY_STABLE_MS: Long = 40L
    @JvmField var INTAKE_SLOT_OCCUPANCY_STABLE_MS: Long = 90L
    @JvmField var EMPTY_STABLE_MS: Long = 300L
    @JvmField var FEEDING_EMPTY_STABLE_MS: Long = 70L

    private val bb1 by lazy { ActiveOpMode.hardwareMap.get(DigitalChannel::class.java, "bb1") }
    private val bb2 by lazy { ActiveOpMode.hardwareMap.get(DigitalChannel::class.java, "bb2") }
    private val bb3 by lazy { ActiveOpMode.hardwareMap.get(DigitalChannel::class.java, "bb3") }
    private val bb4 by lazy { ActiveOpMode.hardwareMap.get(DigitalChannel::class.java, "bb4") }
    private val bb5 by lazy { ActiveOpMode.hardwareMap.get(DigitalChannel::class.java, "bb5") }
    private val bb6 by lazy { ActiveOpMode.hardwareMap.get(DigitalChannel::class.java, "bb6") }

    private val slot1 = SlotLatch()
    private val slot2 = SlotLatch()
    private val slot3 = SlotLatch()
    private var lastRefreshTime: Long = 0L
    var isFullHeld: Boolean = false
        private set
    var cachedBallCount: Int = 0
        private set
    var cachedPos1Occupied: Boolean = false
        private set
    var cachedPos2Occupied: Boolean = false
        private set
    var cachedPos3Occupied: Boolean = false
        private set
    var cachedBb1State: Boolean = true
        private set
    var cachedBb2State: Boolean = true
        private set
    var cachedBb3State: Boolean = true
        private set
    var cachedBb4State: Boolean = true
        private set
    var cachedBb5State: Boolean = true
        private set
    var cachedBb6State: Boolean = true
        private set
    val ballCount: Int get() = refreshBallCount()

    fun refreshBallCount(minIntervalMs: Long = 0L): Int {
        if (isFullHeld && !Rollers.isFeeding) return cachedBallCount
        if (isFullHeld) releaseFullHold()

        val now = System.currentTimeMillis()
        if (minIntervalMs > 0L && now - lastRefreshTime < minIntervalMs) {
            return cachedBallCount
        }

        cachedBb1State = if (BB1_ENABLED) bb1.state else true
        cachedBb2State = bb2.state
        cachedBb3State = bb3.state
        cachedBb4State = bb4.state
        cachedBb5State = bb5.state
        cachedBb6State = bb6.state

        val emptyStableMs = if (Rollers.isFeeding) FEEDING_EMPTY_STABLE_MS else EMPTY_STABLE_MS
        cachedPos1Occupied = slot1.update(rawSlot1Occupied(), now, INTAKE_SLOT_OCCUPANCY_STABLE_MS, emptyStableMs)
        cachedPos2Occupied = slot2.update(!cachedBb3State || !cachedBb4State, now, OCCUPANCY_STABLE_MS, emptyStableMs)
        cachedPos3Occupied = slot3.update(!cachedBb5State || !cachedBb6State, now, OCCUPANCY_STABLE_MS, emptyStableMs)

        cachedBallCount = when {
            cachedPos1Occupied && cachedPos2Occupied && cachedPos3Occupied -> 3
            cachedPos2Occupied && cachedPos3Occupied -> 2
            cachedPos3Occupied -> 1
            else -> 0
        }
        lastRefreshTime = now
        return cachedBallCount
    }

    fun holdFull() {
        isFullHeld = true
        slot1.forceOccupied()
        slot2.forceOccupied()
        slot3.forceOccupied()
        cachedBallCount = 3
        cachedPos1Occupied = true
        cachedPos2Occupied = true
        cachedPos3Occupied = true
    }

    fun releaseFullHold() {
        isFullHeld = false
        lastRefreshTime = 0L
    }

    fun clearStoredBalls() {
        isFullHeld = false
        slot1.reset()
        slot2.reset()
        slot3.reset()
        cachedBallCount = 0
        cachedPos1Occupied = false
        cachedPos2Occupied = false
        cachedPos3Occupied = false
        lastRefreshTime = 0L
    }

    override fun initialize() {
        clearStoredBalls()
        lastRefreshTime = 0L
        cachedBb1State = true
        cachedBb2State = true
        cachedBb3State = true
        cachedBb4State = true
        cachedBb5State = true
        cachedBb6State = true
    }

    private fun rawSlot1Occupied(): Boolean =
        !cachedBb2State || (BB1_ENABLED && !cachedBb1State)

    private class SlotLatch {
        var occupied: Boolean = false
            private set
        private var occupiedStartTime: Long = -1L
        private var emptyStartTime: Long = -1L

        fun update(rawOccupied: Boolean, now: Long, occupyStableMs: Long, emptyStableMs: Long): Boolean {
            if (rawOccupied) {
                emptyStartTime = -1L
                if (!occupied) {
                    if (occupiedStartTime == -1L) occupiedStartTime = now
                    if (now - occupiedStartTime >= occupyStableMs) {
                        occupied = true
                        occupiedStartTime = -1L
                    }
                }
            } else {
                occupiedStartTime = -1L
                if (occupied) {
                    if (emptyStartTime == -1L) emptyStartTime = now
                    if (now - emptyStartTime >= emptyStableMs) {
                        occupied = false
                        emptyStartTime = -1L
                    }
                }
            }
            return occupied
        }

        fun reset() {
            occupied = false
            occupiedStartTime = -1L
            emptyStartTime = -1L
        }

        fun forceOccupied() {
            occupied = true
            occupiedStartTime = -1L
            emptyStartTime = -1L
        }
    }
}

object Stupid {  }
