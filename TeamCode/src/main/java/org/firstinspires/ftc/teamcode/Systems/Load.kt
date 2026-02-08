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
        getShootTripleCommand(1.2),
        getShootTripleCommand(0.85)
    )

    fun getShootTripleCommand(waitTime: Double): Command =
        SequentialGroup(
            InstantCommand {
                Rollers.run(1.0,1.0)
                Rollers.unlockShooter()
            },
            Delay(waitTime),
            InstantCommand {
                Rollers.stop()
                Rollers.lockShooter()
            }
        )
}