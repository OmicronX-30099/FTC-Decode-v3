package org.firstinspires.ftc.teamcode.Autonomous

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.Constants.PedroConstants
import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.TransferSubsystem
import org.firstinspires.ftc.teamcode.Systems.ShooterSystem
import org.firstinspires.ftc.teamcode.Systems.PassiveSystem
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.currAlliance
import org.firstinspires.ftc.teamcode.Util.includePedro
import kotlin.math.PI


@Autonomous(name = "Blue Far 12 ball Auto", group = "Blue 12 autos", preselectTeleOp = "Blue TeleOp - DP")
class BlueAuto12Far: NextFTCOpMode() {
    init {
        addSubsystems(ShooterSystem, PassiveSystem)
        includePedro(PedroConstants::createFollower)
    }
    private var paths: Array<PathChain> = arrayOf()

    override fun onInit() {
        currAlliance = Alliance.BLUE
    }

    override fun onStartButtonPressed() {
        follower.setStartingPose(Pose(63.0,9.7, PI/2.0))
        buildPaths()
        val main = SequentialGroup(
            FollowPath(paths[0],true, 0.75),
            PassiveSystem.shootTripleCommand,
            InstantCommand {
                IntakeSubsystem.intake(1.0)
                TransferSubsystem.transfer(0.35)
            },
            FollowPath(paths[1],true, 1.0),
            InstantCommand {
                IntakeSubsystem.intake(0.0)
                TransferSubsystem.transfer(0.0)
            },
            FollowPath(paths[2],true, 1.0),
            PassiveSystem.shootTripleCommand,
            InstantCommand {
                IntakeSubsystem.intake(1.0)
                TransferSubsystem.transfer(0.35)
            },
            FollowPath(paths[3],true, 1.0),
            InstantCommand {
                IntakeSubsystem.intake(0.0)
                TransferSubsystem.transfer(0.0)
            },
            FollowPath(paths[4],true, 1.0),
            PassiveSystem.shootTripleCommand,
            InstantCommand {
                IntakeSubsystem.intake(1.0)
                TransferSubsystem.transfer(0.35)
            },
            FollowPath(paths[5],true, 1.0),
            InstantCommand {
                IntakeSubsystem.intake(0.0)
                TransferSubsystem.transfer(0.0)
            },
            FollowPath(paths[6],true, 1.0),
            PassiveSystem.shootTripleCommand,
            FollowPath(paths[7], true, 1.0),
        )
        main.schedule()
    }

    override fun onUpdate() {
        ShooterSystem.updateShooter()
        telemetry.update()
    }

    fun buildPaths() {
        val pushPath = follower.pathBuilder().addPath(
            BezierLine(
                Pose(144.0 - 63.000, 9.700),
                Pose(144.0 - 50.000, 9.700)
            )
        ).setConstantHeadingInterpolation(Math.toRadians(90.0))
        .build()

        val intakePath1 = follower.pathBuilder().addPath(
            BezierCurve(
                Pose(144.0 - 50.000, 9.700),
                Pose(144.0 - 50.000, 35.250),
                Pose(144.0 - 11.500, 35.250)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(90.0), Math.toRadians(180.0))
        .build()

        val shootPath1 = follower.pathBuilder().addPath(
            BezierCurve(
                Pose(144.0 - 11.500, 35.250),
                Pose(144.0 - 47.250, 39.500),
                Pose(144.0 - 58.750, 23.500)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(90.0))
        .build()

        val intakePath2 = follower.pathBuilder().addPath(
            BezierCurve(
                Pose(144.0 - 58.750, 23.500),
                Pose(144.0 - 56.600, 58.750),
                Pose(144.0 - .500, 58.750)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(90.0), Math.toRadians(180.0))
        .build()

        val shootPath2 = follower.pathBuilder().addPath(
            BezierCurve(
                Pose(144.0 - 17.500, 58.750),
                Pose(144.0 - 47.500, 60.175),
                Pose(144.0 - .500, 82.250)
            )
        ).setConstantHeadingInterpolation(Math.toRadians(0.0))
        .build()

        val intakePath3 = follower.pathBuilder().addPath(
            BezierLine(
                Pose(144.0 - 59.500, 82.250),
                Pose(144.0 - 17.500, 82.250)
            )
        ).setConstantHeadingInterpolation(Math.toRadians(0.0))
        .build()

        val shootPath3 = follower.pathBuilder().addPath(
            BezierLine(
                Pose(144.0 - 17.500, 82.250),
                Pose(144.0 - 59.500, 82.250)
            )
        ).setConstantHeadingInterpolation(Math.toRadians(0.0))
        .build()

        val leavePath = follower.pathBuilder().addPath(
            BezierLine(Pose(144.0 - 59.500, 82.250),Pose(144.0 - 35.250, 58.750) ))
            .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(-90.0))
            .build()

        paths += pushPath
        paths += intakePath1
        paths += shootPath1
        paths += intakePath2
        paths += shootPath2
        paths += intakePath3
        paths += shootPath3
        paths += leavePath
    }
}