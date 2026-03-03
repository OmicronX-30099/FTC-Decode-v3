package org.firstinspires.ftc.teamcode.Auto.Red

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.instant
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.Constants.PedroConstants
import org.firstinspires.ftc.teamcode.Systems.Load.BilinearIndexMachine
import org.firstinspires.ftc.teamcode.Systems.Load.Load
import org.firstinspires.ftc.teamcode.Systems.Load.Rollers
import org.firstinspires.ftc.teamcode.Systems.Shooter.Shooter
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.includePedro


@Autonomous(name = "Sorted Red Auto", group = "Autos", preselectTeleOp = "Red TeleOp")
class SortedAuto: NextFTCOpMode() {
    init {
        addSubsystems(Shooter, Load)
        includePedro(PedroConstants::createFollower)
    }

    var paths: Array<PathChain> = arrayOf()
    var altpaths: Array<PathChain> = arrayOf()

    override fun onInit() {
        Shooter.reset()
        ROBOT.currStage = Stage.TELEOP
        ROBOT.currAlliance = Alliance.RED
    }


    val LMR: Command = SequentialGroup(
        instant { Rollers.unlockShooter(); BilinearIndexMachine.unlockTransfer(); Rollers.run(0.6,0.6) },
        Delay(1.0),
        instant { BilinearIndexMachine.toLeft() },
        Delay(0.5),
        instant {
            Rollers.stop()
            Rollers.lockShooter()
        }
    )

    fun sortedIntake(path: PathChain, initDelay: Double) = SequentialGroup(
        instant {
            Rollers.lockShooter()
            BilinearIndexMachine.lockTransfer()
            BilinearIndexMachine.toLeft()
        },
        Delay(initDelay),
        ParallelGroup(
            FollowPath(path),
            instant {
                Rollers.intake(0.6)
            }
        ),
        instant { BilinearIndexMachine.toRight(); Rollers.stop() },
        Delay(0.7),
        instant { Rollers.intake(1.0) },
        Delay(0.8), // ok then lets fix the gate path
        instant { Rollers.stop() }
    )

    override fun onStartButtonPressed() {
        follower.setStartingPose(Pose(79.750, 7.500,Math.toRadians(90.0)))
        buildPaths()
        val main = SequentialGroup(
            FollowPath(paths[0]),
            Delay(0.4),
            Load.shootTripleCommand,
            sortedIntake(paths[1],0.5),
            Delay(0.1),
            FollowPath(paths[2]),
            /*Delay(0.2),
            FollowPath(paths[3]),
            LMR,
            sortedIntake(paths[4], 1.2),*/

        )
        main.schedule()

    }

    fun buildPaths() {
        val shootPreload = follower.pathBuilder()
            .addPath(BezierCurve(Pose(79.750, 7.500),Pose(100.000, 6.000),Pose(82.750, 82.250)))
            .setConstantHeadingInterpolation(Math.toRadians(90.0))
            .build()
        val intakeSpike1 = follower.pathBuilder()
            .addPath(BezierLine(Pose(82.750, 82.250),Pose(123.750, 82.250)))
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val openGate = follower.pathBuilder()
            .addPath(BezierCurve(Pose(123.750, 82.250),Pose(127.000, 82.250),Pose(124.250, 77.250)))
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val shootSet1 = follower.pathBuilder()
            .addPath(BezierLine(Pose(124.250, 77.250),Pose(82.750, 82.250)))
            .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(-10.0))
            .build()
        val intakeSpike2 = follower.pathBuilder()
            .addPath(BezierLine(Pose(82.750, 82.250),Pose(100.750, 35.250)))
            .setLinearHeadingInterpolation(Math.toRadians(-70.0), Math.toRadians(0.0))
            .addPath(BezierLine(Pose(100.750, 35.250),Pose(121.750, 35.250)))
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val shootSet2 = follower.pathBuilder()
            .addPath(BezierCurve(Pose(121.750, 35.250),Pose(98.700, 57.000),Pose(82.750, 82.250)))
            .setTangentHeadingInterpolation()
            .setReversed()
            .build()
        val IntakeSpike3 = follower.pathBuilder()
            .addPath(BezierLine(Pose(82.750, 82.250),Pose(100.750, 58.750)))
            .setLinearHeadingInterpolation(Math.toRadians(-53.0), Math.toRadians(0.0))
            .addPath(BezierLine(Pose(100.750, 58.750),Pose(121.750, 58.750)))
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val shootSet3 = follower.pathBuilder()
            .addPath(BezierLine(Pose(121.750, 58.750),Pose(82.750, 82.250)))
            .setTangentHeadingInterpolation()
            .setReversed()
            .build()
        val leave = follower.pathBuilder()
            .addPath(BezierLine(Pose(82.750, 82.250),Pose(118.000, 66.600)))
            .setConstantHeadingInterpolation(Math.toRadians(-31.0))
            .build()

        paths += shootPreload
        paths += intakeSpike1
        paths += openGate
        paths += shootSet1
        paths += intakeSpike2
        paths += shootSet2
        paths += IntakeSpike3
        paths += shootSet3
        paths += leave
    }

    fun buildAlt() {
        val preload = follower.pathBuilder().addPath(
            BezierLine(
                Pose(34.000, 132.900).mirror(),

                Pose(23.5*4.5-23.0, 23.5*3.5)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(0.0))

            .build()
        val intake1 = follower.pathBuilder()
            .addPath(BezierLine(Pose(57.0, 73.0).mirror(), Pose(23.5*4.5-5.0, 23.5*3.5)))
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .addPath(BezierLine(Pose(23.5*4.5-2.0, 23.5*3.5),Pose(23.5*4.5+16.0,23.5*3.5)))
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val shoot1 = follower.pathBuilder()
            .addPath(BezierLine(Pose(23.5*4.5+16.0,23.5*3.5), Pose(23.5*4.5-23.0, 23.5*3.5)))
            .setConstantHeadingInterpolation(0.0)
            .build()
        altpaths += preload
        altpaths += intake1
        altpaths += shoot1
    }

    override fun onUpdate() {
        Shooter.update()
        telemetry.update()
    }
}