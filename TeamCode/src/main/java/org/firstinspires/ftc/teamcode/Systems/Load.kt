@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Systems

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.conditionals.IfElseCommand
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.SubsystemGroup
import org.firstinspires.ftc.teamcode.Systems.LoadSubsystems.BilinearIndexMachine
import org.firstinspires.ftc.teamcode.Systems.LoadSubsystems.Rollers
import org.firstinspires.ftc.teamcode.Util.ROBOT

object Load: SubsystemGroup(BilinearIndexMachine, Rollers) {
    val shootTripleCommand: Command = IfElseCommand(
        { ROBOT.getDistanceFromGoal() > 110.0},
        shootCommand(1.2,0.55,0.55),
        shootCommand(0.85, 1.0, 1.0)
    )

    fun shootCommand(waitTime: Double, tPow: Double, iPow: Double): Command =
        SequentialGroup(
            InstantCommand {
                Rollers.run(tPow,iPow)
                Rollers.unlockShooter()
            },
            Delay(waitTime),
            InstantCommand {
                Rollers.stop()
                Rollers.lockShooter()
            }
        )
}