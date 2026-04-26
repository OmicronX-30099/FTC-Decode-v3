@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Systems.Load

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.conditionals.IfElseCommand
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.subsystems.SubsystemGroup
import org.firstinspires.ftc.teamcode.Util.ROBOT

object Load: SubsystemGroup(Rollers, BreakBeam) {
    val shootTripleCommand: Command = IfElseCommand(
        { ROBOT.shooterPose().distanceFrom(ROBOT.currAlliance.flywheelGoalPose) > 115.0},
        shootCommand(1.0,0.67,0.67),
        shootCommand(0.5, 0.78, 0.78)
    ).setRequirements(Stupid)

    fun shootCommand(waitTime: Double, tPow: Double, iPow: Double): Command =
        SequentialGroup(
            instant {
                Rollers.isFeeding = true
                Rollers.unlockShooter()
                Delay(0.2)
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
