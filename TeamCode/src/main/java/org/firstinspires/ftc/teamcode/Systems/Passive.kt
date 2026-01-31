package org.firstinspires.ftc.teamcode.Systems

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.conditionals.IfElseCommand
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.SubsystemGroup
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.BIMSubsystem
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.ShooterGateSubsystem
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.TransferSubsystem
import org.firstinspires.ftc.teamcode.Util.currAlliance

object PassiveSystem: SubsystemGroup(BIMSubsystem, IntakeSubsystem, TransferSubsystem,
    ShooterGateSubsystem
) {
    val shootTripleCommand: Command
        get() = IfElseCommand(
            { inFarZone() },
            shootTripleCommandFar,
            shootTripleCommandClose
        )
    val shootTripleCommandClose: Command
        get() = SequentialGroup(
            InstantCommand {
                IntakeSubsystem.intake(1.0)
                TransferSubsystem.transfer(1.0)
                ShooterGateSubsystem.open()
            },
            Delay(0.85),
            InstantCommand {
                ShooterGateSubsystem.block()
                IntakeSubsystem.intake(0.0)
                TransferSubsystem.transfer(0.0)
            }
        )
    val shootTripleCommandFar: Command
        get() = SequentialGroup(
            InstantCommand {
                IntakeSubsystem.intake(1.0)
                TransferSubsystem.transfer(0.6)
                ShooterGateSubsystem.open()
            },
            Delay(1.3),
            InstantCommand {
                ShooterGateSubsystem.block()
                IntakeSubsystem.intake(0.0)
                TransferSubsystem.transfer(0.0)
            }
        )
    internal fun inFarZone(): Boolean {
        if (follower.pose.distanceFrom(currAlliance.goalPose) > 101.0) {
            return true
        } else {
            return false
        }
    }
    val shootSingular: Command
        get() = SequentialGroup(
            InstantCommand {
                IntakeSubsystem.intake(1.0)
                TransferSubsystem.transfer(1.0)
                ShooterGateSubsystem.open()
            },
            Delay(0.25),
            InstantCommand {
                ShooterGateSubsystem.block()
                IntakeSubsystem.intake(0.0)
                TransferSubsystem.transfer(0.0)
            }
        )
}