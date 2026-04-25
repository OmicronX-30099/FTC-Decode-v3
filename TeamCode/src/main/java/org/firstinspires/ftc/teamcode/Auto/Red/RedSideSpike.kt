package org.firstinspires.ftc.teamcode.Auto.Red

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
import org.firstinspires.ftc.teamcode.Systems.Load.BreakBeam.ballCount
import org.firstinspires.ftc.teamcode.Systems.Load.Load
import org.firstinspires.ftc.teamcode.Systems.Load.Rollers
import org.firstinspires.ftc.teamcode.Systems.Shooter.Shooter
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.includePedro
import org.firstinspires.ftc.teamcode.pedroPathing.Constants


@Autonomous(name = "Red Side Spike", group = "Solo Auto", preselectTeleOp = "Red TeleOp")
class RedSideSpike(): NextFTCOpMode() {
    init {
        addSubsystems(Load, Shooter)
        includePedro(Constants::createFollower)
    }

    private var paths: Array<PathChain> = arrayOf()

    override fun onInit() {
        ROBOT.currAlliance = Alliance.RED
        Shooter.reset()
        ROBOT.currStage = Stage.AUTONOMOUS
        follower.poseTracker.resetIMU()
        //resetPinpoint()
    }
    override fun onStartButtonPressed() {
        follower.setStartingPose(Pose(112.000, 133.300, Math.toRadians(-90.0)))
        buildPaths()
        val main = SequentialGroup(
            FollowPath(paths[0]), //shoot preload
            Delay(0.2),
            Load.shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[1]), //go to front of first spike
                InstantCommand { Rollers.run(1.0,1.0) }
            ),
            FollowPath(paths[2]), //intake first spike
            Delay(0.2),
            FollowPath(paths[3]), //shoot first spike
            Delay(0.2),
            Load.shootTripleCommand
        )
        main.schedule()
    }
    fun buildPaths() {
        paths = arrayOf()

        val shootpreload = follower.pathBuilder().addPath(
            BezierLine(
                Pose(112.000, 133.300),

                Pose(112.000, 100.000)
            )
        ).setConstantHeadingInterpolation(Math.toRadians(-90.0))

            .build()

        val frontofspike1 = follower.pathBuilder().addPath(
            BezierLine(
                Pose(112.000, 100.000),

                Pose(118.000, 99.000)
            )
        ).setConstantHeadingInterpolation(Math.toRadians(-90.0))

            .build()

        val intakespike1 = follower.pathBuilder().addPath(
            BezierLine(
                Pose(118.000, 99.000),

                Pose(118.000, 90.000)
            )
        ).setConstantHeadingInterpolation(Math.toRadians(-90.0))

            .build()

        val shootspike1 = follower.pathBuilder().addPath(
            BezierLine(
                Pose(118.000, 90.000),

                Pose(112.000, 100.000)
            )
        ).setConstantHeadingInterpolation(Math.toRadians(-90.0))
            .build()

        paths += shootpreload
        paths += frontofspike1
        paths += intakespike1
        paths += shootspike1


    }
    override fun onUpdate() {
        Shooter.update()
        Rollers.update()
        if(follower.pose != Pose(0.0,0.0,0.0)){
            ROBOT.teleopStartPose = follower.pose
        }
        telemetry.addData("ballcount: ", ballCount)
        telemetry.update()
    }

    override fun onStop() {

    }
}