@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Systems.Load

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.conditionals.IfElseCommand
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.instant
import dev.nextftc.core.subsystems.SubsystemGroup
import com.bylazar.configurables.annotations.Configurable
import org.firstinspires.ftc.teamcode.Systems.Shooter.Shooter
import org.firstinspires.ftc.teamcode.Util.ROBOT

@Configurable
object Load: SubsystemGroup(Rollers, BreakBeam) {
    @JvmField var MAX_SHOT_READY_WAIT_SECONDS: Double = 0.2
    @JvmField var GATE_SETTLE_SECONDS: Double = 0.12

    var shotRequestCount: Int = 0
        private set
    var shotStartCount: Int = 0
        private set
    var isWaitingForShooter: Boolean = false
        private set
    var lastShotWaitMs: Long = 0L
        private set
    private var shotWaitStartMs: Long = 0L

    fun resetShotStats() {
        shotRequestCount = 0
        shotStartCount = 0
        isWaitingForShooter = false
        lastShotWaitMs = 0L
        shotWaitStartMs = 0L
    }

    val shootTripleCommand: Command = IfElseCommand(
        { ROBOT.shooterPose().distanceFrom(ROBOT.currAlliance.flywheelGoalPose) > 115.0},
        shootCommand(0.5,0.67,0.67),
        shootCommand(0.4, 0.84, 0.84)
    ).setRequirements(Stupid)

    fun shootCommand(waitTime: Double, tPow: Double, iPow: Double): Command =
        SequentialGroup(
            instant {
                shotRequestCount++
                isWaitingForShooter = true
                shotWaitStartMs = System.currentTimeMillis()
            },
            WaitUntil { Shooter.flywheelReadyToFeed }.endAfter(MAX_SHOT_READY_WAIT_SECONDS),
            instant {
                shotStartCount++
                isWaitingForShooter = false
                lastShotWaitMs = System.currentTimeMillis() - shotWaitStartMs
                Rollers.isFeeding = true
                Rollers.unlockShooter()
            },
            Delay(GATE_SETTLE_SECONDS),
            instant {
                Rollers.run(tPow,iPow)
            },
            Delay(waitTime),
            instant {
                Rollers.stop()
                Rollers.lockShooter()
                Rollers.isFeeding = false
            }
        ).setRequirements(Stupid)

}
