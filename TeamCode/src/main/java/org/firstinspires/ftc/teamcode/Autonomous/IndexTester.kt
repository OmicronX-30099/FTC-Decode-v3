package org.firstinspires.ftc.teamcode.Autonomous

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.Constants.PedroConstants
import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.BIMSubsystem
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.ShooterGateSubsystem
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.TransferSubsystem
import org.firstinspires.ftc.teamcode.Systems.PassiveSystem
import org.firstinspires.ftc.teamcode.Systems.ShooterSystem
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.currAlliance
import org.firstinspires.ftc.teamcode.Util.includePedro
import kotlin.math.PI

@Autonomous(name = "Index test", group = "Test Autos")
class IndexTester: NextFTCOpMode() {
    init {
        addSubsystems(ShooterSystem, PassiveSystem)
        includePedro(PedroConstants::createFollower)
    }
    private var paths: Array<PathChain> = arrayOf()

    override fun onInit() {
        currAlliance = Alliance.RED
    }

    override fun onStartButtonPressed() {
        buildPaths()
        follower.setStartingPose(Pose(72.000, 72.000, Math.toRadians(90.0)))

        /*val main = SequentialGroup(
                InstantCommand { BIMSubsystem.opengate() },
                InstantCommand {
                    TransferSubsystem.transfer(1.0)
                    ShooterGateSubsystem.open()
                },
                Delay(0.3),
                InstantCommand { BIMSubsystem.loadLeft() },
                Delay(0.4),
                InstantCommand { IntakeSubsystem.intake(1.0) },
                Delay(0.5),
                InstantCommand {
                    IntakeSubsystem.intake(0.0)
                    TransferSubsystem.transfer(0.0)
                    ShooterGateSubsystem.block()
                    BIMSubsystem.closeGate()
                }
        )*/
        /*val main = SequentialGroup(
            InstantCommand { BIMSubsystem.opengate() },
            InstantCommand {
                TransferSubsystem.transfer(1.0)
                ShooterGateSubsystem.open()
            },
            Delay(0.3),
            InstantCommand { BIMSubsystem.loadRight() },
            Delay(0.4),
            InstantCommand { IntakeSubsystem.intake(1.0) },
            Delay(0.5),
            InstantCommand {
                IntakeSubsystem.intake(0.0)
                TransferSubsystem.transfer(0.0)
                ShooterGateSubsystem.block()
                BIMSubsystem.closeGate()
            }
        )*/
        val main = SequentialGroup(
            InstantCommand {
                BIMSubsystem.opengate()
                BIMSubsystem.loadMiddle()
            },
            InstantCommand {
                TransferSubsystem.transfer(1.0)
                IntakeSubsystem.intake(1.0)
                ShooterGateSubsystem.open()
            },
            Delay(0.5),
            InstantCommand {
                BIMSubsystem.loadLeft() },
            Delay(0.4),
            InstantCommand {
                BIMSubsystem.loadRight() },
            Delay(0.5),
            InstantCommand {
                IntakeSubsystem.intake(0.0)
                TransferSubsystem.transfer(0.0)
                ShooterGateSubsystem.block()
                BIMSubsystem.closeGate()
            }


            )
        main.schedule()
    }

    override fun onUpdate() {
        ShooterSystem.updateShooter()
        telemetry.update()
    }

    fun buildPaths() {
        val path: PathChain = follower.pathBuilder()
            .addPath(BezierLine(Pose(72.0,72.0), Pose(72.0,87.0)))
            .setConstantHeadingInterpolation(PI/2)
            .build()

        paths += path
    }
}