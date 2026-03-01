package org.firstinspires.ftc.teamcode.Auto.Red

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
import org.firstinspires.ftc.teamcode.Systems.Load.Load
import org.firstinspires.ftc.teamcode.Systems.Load.Rollers
import org.firstinspires.ftc.teamcode.Systems.Shooter.FlywheelState
import org.firstinspires.ftc.teamcode.Systems.Shooter.Shooter
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.includePedro

@Autonomous(name = "18 Ball - RED", group = "Unsorted Auto", preselectTeleOp = "Red TeleOp")
class Red18BallUnsorted(): NextFTCOpMode() {
    init {
        addSubsystems(Load, Shooter)
        includePedro(PedroConstants::createFollower)
    }

    private var paths: Array<PathChain> = arrayOf()

    override fun onInit() {
        ROBOT.currAlliance = Alliance.RED
        Shooter.enableAutoAim()
        ROBOT.currStage = Stage.AUTONOMOUS
        follower.setStartingPose(Pose(34.0,132.9,Math.toRadians(-180.0)).mirror())
    }
    override fun onStartButtonPressed() {
        follower.setStartingPose(Pose(34.0,132.9,Math.toRadians(-180.0)).mirror())
        buildPaths()
        val main = SequentialGroup(
            FollowPath(paths[0]), //shoot preload
            Delay(0.2),
            Load.shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[1]), //intake second spike mark
                InstantCommand { Rollers.run(1.0,0.25) }
            ),
            ParallelGroup(FollowPath(paths[2]), //shoot second spike mark
                InstantCommand { Rollers.run(0.0,0.5)},
            ),
            Delay(0.2),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[3],true,1.0), //intake gate
                InstantCommand { Rollers.run(0.3, 1.0) }
            ),
            Delay(0.4),
            ParallelGroup(FollowPath(paths[4],true,1.0), //shoot gate
                InstantCommand { Rollers.run(0.0,0.5)},
            ),
            Delay(0.2),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[5]), //intake first spike
                InstantCommand { Rollers.run(0.3, 1.0) }
            ),
            InstantCommand { Rollers.run(0.0,0.5)},
            FollowPath(paths[6]), //shoot first spike
            Delay(0.2),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[3],true,1.0), //intake gate
                InstantCommand { Rollers.run(0.3, 1.0) }
            ),
            Delay(0.4),
            ParallelGroup(
                FollowPath(paths[4],true,1.0), //shoot gate
                InstantCommand { Rollers.run(0.0,0.5)},
            ),
            Delay(0.2),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[7]), //intake third spike
                InstantCommand { Rollers.run(0.3, 1.0) }
            ),
            InstantCommand { Rollers.run(0.0,0.5)},
            FollowPath(paths[8]), //shoot third spike
            Delay(0.2),
            Load . shootTripleCommand,
            FollowPath(paths[9]) //leave
        )
        main.schedule()
    }
    fun buildPaths() {
        paths = arrayOf()

        val shootpreload = follower.pathBuilder().addPath(
            BezierLine(
                Pose(34.000, 132.900).mirror(),

                Pose(57.000, 73.000).mirror()
            )
        ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(-23.0))

            .build()

        val intakesecondspike = follower.pathBuilder().addPath(
            BezierLine(
                Pose(57.000, 73.000).mirror(),

                Pose(15.400, 56.200).mirror()
            )
        ).setTangentHeadingInterpolation()

            .build()

        val shootsecondspike = follower.pathBuilder().addPath(
            BezierLine(
                Pose(15.400, 56.200).mirror(),

                Pose(57.000, 73.000).mirror()
            )
        ).setTangentHeadingInterpolation()
            .setReversed()
            .build()

        val gateintake = follower.pathBuilder().addPath(
            BezierLine(
                Pose(57.000, 73.000).mirror(),

                Pose(10.500, 57.000).mirror()
            )
        ).setLinearHeadingInterpolation(Math.toRadians(-23.0), Math.toRadians(180.0-147.731))
            .addParametricCallback(0.8) {follower.setMaxPower(0.2)}
            .addParametricCallback(0.94) {follower.setMaxPower(1.0)}
            .build()

        val shootgate1 = follower.pathBuilder().addPath(
            BezierLine(
                Pose(10.500, 57.000).mirror(),

                Pose(57.000, 73.000).mirror()
            )
        ).setLinearHeadingInterpolation(Math.toRadians(180-147.731), Math.toRadians(20.0))
            .addParametricCallback(0.05) {follower.setMaxPower(1.0)}
            .build()

        val intakefirstspike = follower.pathBuilder().addPath(
            BezierLine(
                Pose(57.000, 73.000).mirror(),

                Pose(19.000, 85.000).mirror()
            )
        ).setTangentHeadingInterpolation()

            .build()

        val shootfirstspike = follower.pathBuilder().addPath(
            BezierLine(
                Pose(19.000, 85.000).mirror(),

                Pose(57.000, 73.000).mirror()
            )
        ).setTangentHeadingInterpolation()
            .setReversed()
            .build()

        val intakethirdspike = follower.pathBuilder().addPath(
            BezierCurve(
                Pose(57.000, 73.000).mirror(),
                Pose(55.000, 32.000).mirror(),
                Pose(45.000, 32.000).mirror(),
                Pose(17.000, 34.000).mirror()
            )
        ).setLinearHeadingInterpolation(Math.toRadians(18.0), Math.toRadians(0.0))

            .build()

        val shootthirdspike = follower.pathBuilder().addPath(
            BezierLine(
                Pose(17.000, 34.000).mirror(),

                Pose(57.000, 78.000).mirror()
            )
        ).setTangentHeadingInterpolation()
            .setReversed()
            .build()

        val leave = follower.pathBuilder().addPath(
            BezierLine(
                Pose(57.000, 78.000).mirror(),

                Pose(52.500, 72.500).mirror()
            )
        ).setTangentHeadingInterpolation()

            .build()
        paths += shootpreload
        paths += intakesecondspike
        paths += shootsecondspike
        paths += gateintake
        paths += shootgate1
        paths += intakefirstspike
        paths += shootfirstspike
        paths += intakethirdspike
        paths += shootthirdspike
        paths += leave

    }
    override fun onUpdate() {
        Shooter.update()
        telemetry.update()
    }
    override fun onStop() { ROBOT.teleopStartPose = follower.pose }
}