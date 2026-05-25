@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Util

import com.bylazar.configurables.annotations.Configurable
import org.firstinspires.ftc.teamcode.Systems.Load.Load
import org.firstinspires.ftc.teamcode.Systems.Load.Rollers
import org.firstinspires.ftc.teamcode.Systems.Shooter.Flywheel
import org.firstinspires.ftc.teamcode.Systems.Shooter.Shooter
import kotlin.math.abs
import kotlin.math.min

@Configurable
object DrivePowerLimiter {
    @JvmField var ENABLED: Boolean = true
    @JvmField var WAITING_FOR_SHOT_CAP: Double = 0.55
    @JvmField var FEEDING_CAP: Double = 0.60
    @JvmField var INTAKING_WHILE_FLYWHEEL_RECOVERING_CAP: Double = 0.70
    @JvmField var INTAKING_CAP: Double = 0.80
    @JvmField var ACTIVE_POWER_THRESHOLD: Double = 0.2

    var currentCap: Double = 1.0
        private set
    var currentReason: String = "none"
        private set

    fun limit(requestedScalar: Double): Double {
        if (!ENABLED) {
            currentCap = 1.0
            currentReason = "disabled"
            return requestedScalar
        }

        val intakeActive = abs(Rollers.intakePower) > ACTIVE_POWER_THRESHOLD ||
            abs(Rollers.transferPower) > ACTIVE_POWER_THRESHOLD
        val flywheelRecovering = Flywheel.targetVelocity > 500.0 &&
            !Shooter.flywheelReadyToFeed &&
            abs(Shooter.flywheelVelocityError) > Shooter.FEED_READY_TOLERANCE_TPS

        val cap = when {
            Load.isWaitingForShooter -> WAITING_FOR_SHOT_CAP
            Rollers.isFeeding -> FEEDING_CAP
            intakeActive && flywheelRecovering -> INTAKING_WHILE_FLYWHEEL_RECOVERING_CAP
            intakeActive -> INTAKING_CAP
            else -> 1.0
        }

        currentCap = cap
        currentReason = when (cap) {
            WAITING_FOR_SHOT_CAP -> "waiting_for_shot"
            FEEDING_CAP -> "feeding"
            INTAKING_WHILE_FLYWHEEL_RECOVERING_CAP -> "intake_flywheel_recovery"
            INTAKING_CAP -> "intaking"
            else -> "none"
        }
        return min(requestedScalar, cap)
    }

    fun reset() {
        currentCap = 1.0
        currentReason = "none"
    }
}
