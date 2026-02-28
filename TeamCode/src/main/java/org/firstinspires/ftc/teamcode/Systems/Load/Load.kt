@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Systems.Load

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.conditionals.IfElseCommand
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.subsystems.SubsystemGroup
import org.firstinspires.ftc.teamcode.Util.ROBOT

object Load: SubsystemGroup(BilinearIndexMachine, Rollers, BreakBeam) {
    val shootTripleCommand: Command = IfElseCommand(
        { ROBOT.shooterPose().distanceFrom(ROBOT.currAlliance.goalPoses.flywheelGoalPose) > 110.0},
        shootCommand(1.0,0.7,0.7),
        shootCommand(0.6, 1.0, 1.0)
    )

    fun shootCommand(waitTime: Double, tPow: Double, iPow: Double): Command =
        SequentialGroup(
            instant {
                Rollers.run(tPow,iPow)
                Rollers.unlockShooter()
            },
            Delay(waitTime),
            instant {
                Rollers.stop()
                Rollers.lockShooter()
            }
        )
}