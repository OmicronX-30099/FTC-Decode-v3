package org.firstinspires.ftc.teamcode.Systems

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.conditionals.IfElseCommand
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.SubsystemGroup
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.BIM
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.Transfer
import org.firstinspires.ftc.teamcode.Util.SequentialGroup
import org.firstinspires.ftc.teamcode.Util.currAlliance

object Load: SubsystemGroup(BIM, Transfer) {
    val shootTripleCommand: Command
        get() = IfElseCommand(
            { follower.pose.distanceFrom(currAlliance.goalPose) > 110.0 },
            shootTripleFar,
            shootTripleClose
        )
    val shootTripleFar: Command
        get() = shootCommand(0.8,0.45,1.3)
    val shootTripleClose: Command
        get() = shootCommand(1.0,1.0,0.85)
    val shootSingular: Command
        get() = shootCommand(1.0,1.0,0.25)

    internal fun shootCommand(intakePower: Double, transferPower: Double, time: Double): Command
        = SequentialGroup(
            InstantCommand {
                Transfer.intake(intakePower)
                Transfer.transfer(transferPower)
                Transfer.unlockShooter()
            },
            Delay(time),
            InstantCommand {
                Transfer.lockShooter()
                Transfer.intake(0.0)
                Transfer.transfer(0.0)
            }
        )
}