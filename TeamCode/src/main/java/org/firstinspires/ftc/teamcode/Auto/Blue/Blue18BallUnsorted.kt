package org.firstinspires.ftc.teamcode.Auto.Blue

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
import org.firstinspires.ftc.teamcode.Systems.Shooter.Shooter
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.includePedro
import org.firstinspires.ftc.teamcode.Util.resetPinpoint

@Autonomous(name = "18 Ball - BLUE", group = "Unsorted Auto", preselectTeleOp = "Blue TeleOp")
class Blue18BallUnsorted(): NextFTCOpMode() {
    init {
        addSubsystems(Load, Shooter)
        includePedro(PedroConstants::createFollower)
    }

    private var paths: Array<PathChain> = arrayOf()

    override fun onInit() {
        ROBOT.currAlliance = Alliance.BLUE
        Shooter.enableAutoAim()
        ROBOT.currStage = Stage.AUTONOMOUS
        resetPinpoint()
    }

    override fun onStartButtonPressed() {
        follower.setStartingPose(Pose(33.0,132.6875,Math.toRadians(-180.0)))
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
                InstantCommand { Rollers.run(0.0,0.0)},
            ),
            Delay(0.2),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[3],true,1.0), //intake gate
                InstantCommand { Rollers.run(1.0, 0.25) }
            ),
            Delay(0.9),
            ParallelGroup(FollowPath(paths[4],true,1.0), //shoot gate
                InstantCommand { Rollers.run(0.0,0.0)},
            ),
            Delay(0.2),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[3],true,1.0), //intake gate
                InstantCommand { Rollers.run(1.0, 0.25) }
            ),
            Delay(0.9),
            ParallelGroup(
                FollowPath(paths[4],true,1.0), //shoot gate
                InstantCommand { Rollers.run(0.0,0.0)},
            ),
            Delay(0.2),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[5]), //intake first spike
                InstantCommand { Rollers.run(1.0, 0.25) }
            ),
            InstantCommand { Rollers.run(0.0,0.0)},
            FollowPath(paths[6]), //shoot first spike
            Delay(0.2),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[7]), //intake third spike
                InstantCommand { Rollers.run(1.0, 0.25) }
            ),
            InstantCommand { Rollers.run(0.0,0.0)},
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
                Pose(33.0,132.6875),

                Pose(56.000, 73.000)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(-180.0), Math.toRadians(-157.0))

            .build()

        val intakesecondspike = follower.pathBuilder().addPath(
            BezierLine(
                Pose(56.000, 73.000),

                Pose(10.400, 56.200)
            )
        ).setTangentHeadingInterpolation()

            .build()

        val shootsecondspike = follower.pathBuilder().addPath(
            BezierLine(
                Pose(10.400, 56.200),

                Pose(56.000, 73.000)
            )
        ).setTangentHeadingInterpolation()
            .setReversed()
            .build()

        val gateintake = follower.pathBuilder().addPath(
            BezierLine(
                Pose(56.000, 73.000),

                Pose(10.5, 59.5)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(-157.0), Math.toRadians(160.0))
            .addParametricCallback(0.8) {follower.setMaxPower(0.2)}
            .addParametricCallback(0.94) {follower.setMaxPower(1.0)}
            .build()

        val shootgate1 = follower.pathBuilder().addPath(
            BezierLine(
                Pose(10.5, 59.5),

                Pose(56.000, 73.000)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(160.0), Math.toRadians(160.0))
            .addParametricCallback(0.05) {follower.setMaxPower(1.0)}
            .build()

        val intakefirstspike = follower.pathBuilder().addPath(
            BezierLine(
                Pose(56.000, 73.000),

                Pose(19.000, 85.000)
            )
        ).setTangentHeadingInterpolation()

            .build()

        val shootfirstspike = follower.pathBuilder().addPath(
            BezierLine(
                Pose(19.000, 85.000),

                Pose(56.000, 73.000)
            )
        ).setTangentHeadingInterpolation()
            .setReversed()
            .build()

        val intakethirdspike = follower.pathBuilder().addPath(
            BezierCurve(
                Pose(56.000, 73.000),
                Pose(55.000, 32.000),
                Pose(45.000, 32.000),
                Pose(17.000, 34.000)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(162.0), Math.toRadians(180.0))

            .build()

        val shootthirdspike = follower.pathBuilder().addPath(
            BezierLine(
                Pose(17.000, 34.000),

                Pose(56.000, 76.000)
            )
        ).setTangentHeadingInterpolation()
            .setReversed()
            .build()

        val leave = follower.pathBuilder().addPath(
            BezierLine(
                Pose(56.000, 76.000),

                Pose(52.500, 72.500)
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
        ROBOT.teleopStartPose = follower.pose
        telemetry.addData("follower", follower.pose)
        telemetry.update()
    }

    override fun onStop() {
        ROBOT.teleopStartPose = follower.pose
    }
}