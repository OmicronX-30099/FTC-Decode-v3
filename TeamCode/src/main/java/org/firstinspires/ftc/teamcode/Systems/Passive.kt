package org.firstinspires.ftc.teamcode.Systems

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.SubsystemGroup
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.BIMSubsystem
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.ShooterGateSubsystem
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.TransferSubsystem

object PassiveSystem: SubsystemGroup(BIMSubsystem, IntakeSubsystem, TransferSubsystem,
    ShooterGateSubsystem
) {
    val shootTripleCommand: Command
        get() = SequentialGroup(
            InstantCommand {
                IntakeSubsystem.intake(1.0)
                TransferSubsystem.transfer(0.75)
                ShooterGateSubsystem.open()
            },
            Delay(0.8),
            InstantCommand {
                ShooterGateSubsystem.block()
                IntakeSubsystem.intake(0.0)
                TransferSubsystem.transfer(0.0)
            }
        )
    val shootSingular: Command
        get() = SequentialGroup(
            InstantCommand {
                IntakeSubsystem.intake(1.0)
                TransferSubsystem.transfer(1.0)
                ShooterGateSubsystem.open()
            },
            Delay(0.5),
            InstantCommand {
                ShooterGateSubsystem.block()
                IntakeSubsystem.intake(0.0)
                TransferSubsystem.transfer(0.0)
            }
        )
}