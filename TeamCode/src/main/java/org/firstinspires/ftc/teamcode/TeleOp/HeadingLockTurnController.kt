@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.TeleOp

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.control.PIDFCoefficients
import com.pedropathing.math.MathFunctions
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.math.abs

@Configurable
object HeadingLockTurnTuning {
    @JvmField var TARGET_HEADING_DEGREES: Double = 25.0
    @JvmField var SECONDARY_PIDF_THRESHOLD_DEGREES: Double = 30.0
    @JvmField var MAX_TURN_POWER: Double = 1.0
    @JvmField var MANUAL_TURN_SCALE: Double = 0.5
    @JvmField var TURN_STICK_DEADBAND: Double = 0.0
}

class HeadingLockTurnController {
    private var lastHeadingError: Double = 0.0
    private var lastUpdateTimeNanos: Long = 0L

    var enabled: Boolean = false
        private set

    fun enableIfTurnStickCentered(rawTurnStickX: Double) {
        if (abs(rawTurnStickX) <= HeadingLockTurnTuning.TURN_STICK_DEADBAND) {
            enabled = true
            resetControllerState(currentHeadingError())
        }
    }

    fun disable() {
        enabled = false
        resetControllerState()
    }

    fun turnPower(rawTurnStickX: Double): Double {
        if (abs(rawTurnStickX) > HeadingLockTurnTuning.TURN_STICK_DEADBAND) {
            enabled = false
            resetControllerState()
            return manualTurnPower(rawTurnStickX)
        }

        return if (enabled) {
            headingLockTurnPower()
        } else {
            manualTurnPower(rawTurnStickX)
        }
    }

    private fun manualTurnPower(rawTurnStickX: Double): Double =
        -rawTurnStickX * HeadingLockTurnTuning.MANUAL_TURN_SCALE

    private fun headingLockTurnPower(): Double {
        val targetHeading = Math.toRadians(HeadingLockTurnTuning.TARGET_HEADING_DEGREES)
        val headingError = currentHeadingError()
        val now = System.nanoTime()
        val dt = ((now - lastUpdateTimeNanos) / 1_000_000_000.0).takeIf { it > 0.0 } ?: 0.0
        val errorDerivative = if (dt > 0.0) (headingError - lastHeadingError) / dt else 0.0
        val turnDirection = MathFunctions.getTurnDirection(follower.pose.heading, targetHeading)
        val secondaryPidfThreshold = Math.toRadians(HeadingLockTurnTuning.SECONDARY_PIDF_THRESHOLD_DEGREES)
        val coeffs = if (abs(headingError) < secondaryPidfThreshold) {
            PIDFCoefficients(0.1, 0.0, 0.1, 0.02)
        } else {
            PIDFCoefficients(0.6,0.0,0.1,0.02)
        }

        lastHeadingError = headingError
        lastUpdateTimeNanos = now

        return (headingError * (coeffs.P) + errorDerivative * coeffs.D + turnDirection * coeffs.F)
            .coerceIn(-HeadingLockTurnTuning.MAX_TURN_POWER, HeadingLockTurnTuning.MAX_TURN_POWER)
    }

    private fun currentHeadingError(): Double {
        val targetHeading = Math.toRadians(HeadingLockTurnTuning.TARGET_HEADING_DEGREES)
        return MathFunctions.normalizeAngleSigned(targetHeading - follower.pose.heading)
    }

    private fun resetControllerState(headingError: Double = 0.0) {
        lastHeadingError = headingError
        lastUpdateTimeNanos = System.nanoTime()
    }
}
