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
        { ROBOT.shooterPose().distanceFrom(ROBOT.currAlliance.flywheelGoalPose) > 115.0},
        shootCommand(1.1,0.55,0.55),
        shootCommand(0.65, 0.9, 0.9)
    )

    /*val LMR: Command = SequentialGroup(
        instant { Rollers.unlockShooter(); BilinearIndexMachine.unlockTransfer(); Rollers.run(0.6,0.6) },
        Delay(1.0),
        instant { BilinearIndexMachine.toLeft() },
        Delay(0.5),
        instant {
            Rollers.stop()
            Rollers.lockShooter()
        }

    )*/

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