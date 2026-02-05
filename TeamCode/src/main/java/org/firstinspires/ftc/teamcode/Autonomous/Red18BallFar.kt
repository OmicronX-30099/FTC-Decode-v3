package org.firstinspires.ftc.teamcode.Autonomous

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.Constants.PedroConstants
import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.BreakBeamSubsystem
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.TransferSubsystem
import org.firstinspires.ftc.teamcode.Systems.PassiveSystem
import org.firstinspires.ftc.teamcode.Systems.ShooterSystem
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.currAlliance
import org.firstinspires.ftc.teamcode.Util.includePedro
import kotlin.math.PI

@Autonomous(name = "Red 18 Far", group = "18 ball", preselectTeleOp = "Red TeleOp - DP")
class Red18BallFar: NextFTCOpMode() {
    init {
        addSubsystems(ShooterSystem, PassiveSystem)
        includePedro(PedroConstants::createFollower)
    }
    private var paths: Array<PathChain> = arrayOf()

    override fun onInit() {
        currAlliance = Alliance.RED
    }

    override fun onWaitForStart() {
        BreakBeamSubsystem.update()
    }
    override fun onStartButtonPressed() {
        buildPaths()
        follower.setStartingPose(Pose(56.000, 9.700, PI/2).mirror(141.5))
        val cycleCommand: Command =  SequentialGroup(
            InstantCommand {
                IntakeSubsystem.intake(1.0)
                TransferSubsystem.transfer(0.35)
            },
            FollowPath(paths[2],holdEnd = true),
            Delay(2.0),
            ParallelGroup(
                FollowPath(paths[3]),
                SequentialGroup(
                    Delay(0.5),
                    InstantCommand {
                        IntakeSubsystem.intake(0.0)
                        TransferSubsystem.transfer(0.0)
                    }
                ),
            ),
            PassiveSystem.shootTripleCommand
        )
        val main = SequentialGroup(
            PassiveSystem.shootTripleCommand,
            InstantCommand {
                IntakeSubsystem.intake(1.0)
                TransferSubsystem.transfer(0.35)
            },
            FollowPath(paths[0]),
            ParallelGroup(
                FollowPath(paths[1]),
                SequentialGroup(
                    Delay(0.5),
                    InstantCommand {
                        IntakeSubsystem.intake(0.0)
                        TransferSubsystem.transfer(0.0)
                    }
                )
            ),
            PassiveSystem.shootTripleCommand,
            cycleCommand,
            cycleCommand,
            cycleCommand,
            cycleCommand,
            FollowPath(paths[4])
        )
        main.schedule()
    }

    override fun onUpdate() {
        ShooterSystem.updateShooter()
        BreakBeamSubsystem.update()
        telemetry.update()
    }

    fun buildPaths() {
        val intakePath = follower.pathBuilder().addPath(
            BezierCurve(
                Pose(56.000, 9.700).mirror(141.5),
                Pose(43.000, 25.400).mirror(141.5),
                Pose(17.000, 35.250).mirror(141.5)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(90.0), Math.toRadians(0.0))
        .build()

        val shootPath = follower.pathBuilder().addPath(
            BezierLine(
                Pose(17.000, 35.250).mirror(141.5),
                Pose(58.750, 18.500).mirror(141.5)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(20.0))
        .build()

        val intakeCyclePath = follower.pathBuilder().addPath(
            BezierLine(
                Pose(58.750, 18.500).mirror(141.5),
                Pose(128.8, 60.1)
            )
        )
        .setLinearHeadingInterpolation(Math.toRadians(20.0), 0.635)
        .build()

        val shootCyclePath = follower.pathBuilder().addPath(
            BezierLine(
                Pose(128.8, 60.1),
                Pose(58.750, 18.500).mirror(141.5)
            )
        )
            .setLinearHeadingInterpolation(0.635, Math.toRadians(20.0))
        .build()
        val leavePath = follower.pathBuilder().addPath(
            BezierLine(
                Pose(58.750, 18.500).mirror(141.5),
                Pose(25.000, 44.300).mirror(141.5)
            )
        )
        .setConstantHeadingInterpolation(Math.toRadians(20.0))
        .build()
        paths += intakePath
        paths += shootPath
        paths += intakeCyclePath
        paths += shootCyclePath
        paths += leavePath
    }
}