package org.firstinspires.ftc.teamcode.Autonomoous.Red

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
import org.firstinspires.ftc.teamcode.Systems.Load
import org.firstinspires.ftc.teamcode.Systems.LoadSubsystems.Rollers
import org.firstinspires.ftc.teamcode.Systems.Miscellaneous
import org.firstinspires.ftc.teamcode.Systems.Shooter
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.includePedro

@Autonomous(name = "Unsorted 18-ball Red", group = "Unsorted Auto", preselectTeleOp = "Red TeleOp")
class Unsorted18AutoRed(): NextFTCOpMode() {
    init {
        addSubsystems(Load, Shooter, Miscellaneous)
        includePedro(PedroConstants::createFollower)
    }

    private var paths: Array<PathChain> = arrayOf()
    
    override fun onInit() {
        ROBOT.currAlliance = Alliance.RED
        ROBOT.currStage = Stage.AUTONOMOUS
        ROBOT.currStage.useFlywheelVel = true
    }
    override fun onStartButtonPressed() {
        follower.setStartingPose(Pose(0.0,0.0,Math.toRadians(-90.0)))
        buildPaths()
        val main = SequentialGroup(
            FollowPath(paths[0]),
            Delay(0.1),
            Load.shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[1]),
                InstantCommand { Rollers.run(0.3, 1.0) }
            ),
            Delay(0.1),
            FollowPath(paths[2]),
            Delay(0.2),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[3]),
                InstantCommand { Rollers.run(0.3, 1.0) }
            ),
            Delay(1.6),
            FollowPath(paths[4]),
            Delay(0.2),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[5]),
                InstantCommand { Rollers.run(0.3, 1.0) }
            ),
            Delay(0.1),
            FollowPath(paths[6]),
            Delay(0.2),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[7]),
                InstantCommand { Rollers.run(0.3, 1.0) }
            ),
            Delay(0.1),
            FollowPath(paths[8]),
            Delay(0.2),
            Load . shootTripleCommand,
        )
        main.schedule()
    }
    fun buildPaths() {
        paths = arrayOf()
        val startPose: Pose = Pose(0.0,0.0,Math.toRadians(-90.0))
        val shootPose: Pose = Pose(84.0,85.0)
        val middleIntakeEndPose: Pose = Pose(128.5,56.0,Math.toRadians(-20.0))
        val middleIntakeControl: Pose = Pose(103.75, 65.5)

        val gateIntakePose: Pose = Pose(127.35, 61.475, 0.5739)

        val topIntakePose: Pose = Pose(120.0,85.0,0.0)

        val lastIntakePose: Pose = Pose(125.0,33.0,Math.toRadians(-15.0))
        val lastIntakeControl: Pose = Pose(90.0,40.0)

        val lastShootPose: Pose = Pose(81.0,101.0,Math.toRadians(-90.0))

        val path1 = follower.pathBuilder()
            .addPath(BezierLine(startPose, shootPose))
            .setLinearHeadingInterpolation(startPose.heading, Math.toRadians(-40.0))
            .build()
        val path2 = follower.pathBuilder()
            .addPath(BezierCurve(shootPose, middleIntakeControl, middleIntakeEndPose))
            .setLinearHeadingInterpolation(Math.toRadians(-40.0),middleIntakeEndPose.heading)
            .build()
        val path3 = follower.pathBuilder()
            .addPath(BezierLine(middleIntakeControl, shootPose))
            .setConstantHeadingInterpolation(Math.toRadians(-40.0))
            .build()
        val path4 = follower.pathBuilder()
            .addPath(BezierLine(shootPose, gateIntakePose))
            .setLinearHeadingInterpolation(Math.toRadians(-30.0),gateIntakePose.heading)
            .build()
        val path5 = follower.pathBuilder()
            .addPath(BezierLine(gateIntakePose, shootPose))
            .setLinearHeadingInterpolation(gateIntakePose.heading, Math.toRadians(-30.0))
            .build()
        val path6 = follower.pathBuilder()
            .addPath(BezierLine(shootPose, topIntakePose))
            .setConstantHeadingInterpolation(0.0)
            .build()
        val path7 = follower.pathBuilder()
            .addPath(BezierLine(topIntakePose, shootPose))
            .setLinearHeadingInterpolation(0.0,Math.toRadians(-45.0))
            .build()
        val path8 = follower.pathBuilder()
            .addPath(BezierCurve(shootPose, lastIntakeControl, lastIntakePose))
            .setLinearHeadingInterpolation(Math.toRadians(-90.0),Math.toRadians(-15.0))
            .build()
        val path9 = follower.pathBuilder()
            .addPath(BezierCurve(lastIntakePose, Pose(83.0,68.5),lastShootPose))
            .setLinearHeadingInterpolation(Math.toRadians(-15.0),lastShootPose.heading)
            .build()
        paths += path1
        paths += path2
        paths += path3
        paths += path4
        paths += path5
        paths += path6
        paths += path7
        paths += path8
        paths += path9

    }
}
