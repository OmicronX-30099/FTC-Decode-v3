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
        shootCommand(1.2,0.55,0.55),
        shootCommand(0.7, 1.0, 1.0)
    ).setRequirements(Stupid)

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
        ).setRequirements(Stupid)

    val LMR: Command = SequentialGroup(
        instant { Rollers.unlockShooter(); BilinearIndexMachine.unlockTransfer(); Rollers.run(1.0,0.65) },
        Delay(0.8),
        instant { BilinearIndexMachine.toLeft() },
        Delay(0.8),
        instant {
            Rollers.stop()
            Rollers.lockShooter()
        }
    ).setRequirements(Stupid)
    val RLM: Command = SequentialGroup(
        instant { Rollers.unlockShooter(); BilinearIndexMachine.unlockTransfer(); Rollers.transfer(0.7); BilinearIndexMachine.toLeft() },
        Delay(0.4),
        instant { BilinearIndexMachine.toRight() },
        Delay(0.7),
        instant { Rollers.intake(1.0) },
        Delay(0.9),
        instant { Rollers.stop() }
    ).setRequirements(Stupid)
    val RML: Command = SequentialGroup(
        instant { Rollers.unlockShooter(); BilinearIndexMachine.unlockTransfer(); Rollers.run(1.0,0.7); BilinearIndexMachine.toLeft() },
        Delay(1.0),
        instant { BilinearIndexMachine.toRight() },
        Delay(0.6),
        instant { Rollers.stop() }
    ).setRequirements(Stupid)
    val MRL: Command = SequentialGroup(
        instant { Rollers.unlockShooter(); BilinearIndexMachine.unlockTransfer(); Rollers.run(1.0,0.7); BilinearIndexMachine.split() },
        Delay(0.8),
        instant { BilinearIndexMachine.toLeft() },
        Delay(0.6),
        instant { BilinearIndexMachine.toRight() },
        Delay(0.7),
        instant { Rollers.stop() }
    ).setRequirements(Stupid)
}
