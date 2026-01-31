package org.firstinspires.ftc.teamcode.Autonomous

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.Constants.PedroConstants
import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.TransferSubsystem
import org.firstinspires.ftc.teamcode.Systems.PassiveSystem
import org.firstinspires.ftc.teamcode.Systems.ShooterSystem
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.currAlliance
import org.firstinspires.ftc.teamcode.Util.includePedro
import kotlin.math.PI


@Autonomous(name = "Tester", group = "Tests")
class Tester: NextFTCOpMode() {
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
        follower.setStartingPose(Pose(87.500, 8.000, Math.toRadians(0.0)))
        val main = SequentialGroup(
            FollowPath(paths[0]), //go to first shoot
            Delay(0.5),
            PassiveSystem.shootTripleCommand,
            InstantCommand {
                IntakeSubsystem.intake(1.0)
                TransferSubsystem.transfer(0.25)
            },
            FollowPath(paths[1]), //intake first spike
            InstantCommand {
                IntakeSubsystem.intake(0.0)
                TransferSubsystem.transfer(0.0)
            },
            //FollowPath(paths[2]),//turn
            FollowPath(paths[3]),//go to shoot
            PassiveSystem.shootTripleCommand,
            InstantCommand {
                IntakeSubsystem.intake(1.0)
                TransferSubsystem.transfer(0.25)
            },
            Delay(1.0),
            FollowPath(paths[4]),//intake second p1
            FollowPath(paths[5]),//intake second p2
            FollowPath(paths[6]),//intake second p3
            InstantCommand {
                IntakeSubsystem.intake((0.0))
                TransferSubsystem.transfer((0.0))
            },
            FollowPath(paths[7]),//shoot second
            Delay(0.25),
            PassiveSystem.shootTripleCommand,
            InstantCommand {
                IntakeSubsystem.intake((1.0))
                TransferSubsystem.transfer((0.25))
            },
            FollowPath(paths[8]), //intake third p1
            FollowPath(paths[9]), //intake third p2
            InstantCommand {
                IntakeSubsystem.intake((0.0))
                TransferSubsystem.transfer((0.0))
            },
            FollowPath(paths[10]), //shoot 3rd
            Delay(0.25),
            PassiveSystem.shootTripleCommand,
            FollowPath(paths[11]), //leave
        )
        main.schedule()
    }

    override fun onUpdate() {
        ShooterSystem.updateShooter()
        telemetry.update()
    }

    fun buildPaths() {
        val shootpreload = follower //path0
            .pathBuilder()
            .addPath(
                BezierLine(Pose(87.500, 8.000), Pose(87.500, 20.000))
            )
            .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(30.0))
            .build()

        val firstintake = follower //path1
            .pathBuilder()
            .addPath(
                BezierCurve(
                    Pose(87.500, 20.000),
                    Pose(100.000, 35.000),
                    Pose(130.000, 35.000)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(30.0), Math.toRadians(0.0))
            .build()

        val turn = follower //path2
            .pathBuilder()
            .addPath(
                BezierLine(Pose(130.000,35.000), Pose(130.000,35.000))
            )
            .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(20.0))
            .build()

        val shootfirst = follower //path3
            .pathBuilder()
            .addPath(
                BezierLine(Pose(130.000, 35.000), Pose(87.500, 20.000))
            )
            .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(20.0))
            .build()

        val intakesecondp1 = follower //path4
            .pathBuilder()
            .addPath(
                BezierLine(Pose(87.500, 20.000), Pose(133.000, 27.000))
            )
            .setLinearHeadingInterpolation(Math.toRadians(20.0), Math.toRadians(10.0))
            .build()

        val intakesecondp2 = follower //path5
            .pathBuilder()
            .addPath(
                BezierLine(Pose(133.000, 27.000), Pose(123.000, 25.000))
            )
            .setLinearHeadingInterpolation(Math.toRadians(10.0), Math.toRadians(0.0))
            .build()

        val intakesecondp3 = follower //path6
            .pathBuilder()
            .addPath(
                BezierLine(Pose(123.000, 25.000), Pose(133.000, 25.000))
            )
            .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(0.0))
            .build()

        val shootsecond = follower //path7
            .pathBuilder()
            .addPath(
                BezierLine(Pose(133.000, 25.000), Pose(85.000, 25.000))
            )
            .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(0.0))
            .build()

        val intake3rdp1 = follower //path8
            .pathBuilder()
            .addPath(
                BezierLine(Pose(85.000, 25.000), Pose(130.000, 11.000))
            )
            .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(-15.0))
            .build()

        val intake3rdp2 = follower //path9
            .pathBuilder()
            .addPath(
                BezierCurve(
                    Pose(130.000, 11.000),
                    Pose(128.000, 10.000),
                    Pose(133.000, 9.000)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(-15.0), Math.toRadians(0.0))
            .build()

        val shootthird = follower //path10
            .pathBuilder()
            .addPath(
                BezierLine(Pose(133.000, 9.000), Pose(90.000, 9.000))
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()

        val leave = follower //path11
            .pathBuilder()
            .addPath(
                BezierLine(Pose(90.000, 9.000), Pose(110.000, 9.000))
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        paths += shootpreload
        paths += firstintake
        paths += turn
        paths += shootfirst
        paths += intakesecondp1
        paths += intakesecondp2
        paths += intakesecondp3
        paths += shootsecond
        paths += intake3rdp1
        paths += intake3rdp2
        paths += shootthird
        paths += leave
    }
}