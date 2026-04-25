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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.Systems.Load.Load
import org.firstinspires.ftc.teamcode.Systems.Load.Rollers
import org.firstinspires.ftc.teamcode.Systems.Shooter.Shooter
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.includePedro
import org.firstinspires.ftc.teamcode.Util.resetPinpoint

@Autonomous(name = "18 Ball - RED - LANCERS", group = "Collab Auto", preselectTeleOp = "Red TeleOp")
class Red18BallAlliance(): NextFTCOpMode() {
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
    }
    override fun onStartButtonPressed() {
        follower.setStartingPose(Pose(112.0,130.75,Math.toRadians(90.0)))
        buildPaths()
        val main = SequentialGroup(
            FollowPath(paths[0]), //shoot preload
            Delay(0.2),
            Load.shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[1]), //intake second spike mark
                InstantCommand { Rollers.run(1.0,0.67) }
            ),
            ParallelGroup(FollowPath(paths[2]), //shoot second spike mark
            ),
            Delay(0.2),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[3]), //intake gate
                InstantCommand { Rollers.run(1.0, 0.67) }
            ),
            Delay(1.5),
            ParallelGroup(FollowPath(paths[4],true,1.0), //shoot gate
                Delay(0.5),
                InstantCommand { Rollers.run(0.0,0.0)}
            ),
            Delay(0.2),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[3]), //intake gate
                InstantCommand { Rollers.run(1.0, 0.67) }
            ),
            Delay(1.5),
            ParallelGroup(
                FollowPath(paths[4],true,1.0), //shoot gate
                Delay(0.5),
                InstantCommand { Rollers.run(0.0,0.0)}
            ),
            Delay(0.2),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[3]), //intake gate
                InstantCommand { Rollers.run(1.0, 0.67) }
            ),
            Delay(1.5),
            ParallelGroup(
                FollowPath(paths[4],true,1.0), //shoot gate
                Delay(0.5),
                InstantCommand { Rollers.run(0.0,0.0)}
            ),
            Delay(0.2),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[5]), //intake first spike
                InstantCommand { Rollers.run(1.0, 0.67) }
            ),
            FollowPath(paths[6]), //shoot first spike
            Delay(0.2),
            Load . shootTripleCommand,
        )
        main.schedule()
    }
    fun buildPaths() {
        paths = arrayOf()

        val shootpreload = follower.pathBuilder().addPath(
            BezierLine(
                Pose(30.000, 130.750).mirror(142.0),

                Pose(57.000, 73.000).mirror(142.0)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(90.0), Math.toRadians(-23.0))

            .build()

        val intakesecondspike = follower.pathBuilder().addPath(
            BezierLine(
                Pose(57.000, 73.000).mirror(142.0),

                Pose(15.400, 56.200).mirror(142.0)
            )
        ).setTangentHeadingInterpolation()

            .build()

        val shootsecondspike = follower.pathBuilder().addPath(
            BezierLine(
                Pose(15.400, 56.200).mirror(142.0),

                Pose(57.000, 73.000).mirror(142.0)
            )
        ).setTangentHeadingInterpolation()
            .setReversed()
            .build()

        val gateintake = follower.pathBuilder().addPath(
            BezierLine(
                Pose(57.000, 73.000).mirror(142.0),

                Pose(12.300, 57.500).mirror(142.0)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(-23.0), Math.toRadians(180.0-157.0))
            .addParametricCallback(0.5) {follower.setMaxPower(0.5)}
            .build()

        val shootgate1 = follower.pathBuilder().addPath(
            BezierLine(
                Pose(12.300, 57.500).mirror(142.0),

                Pose(57.000, 73.000).mirror(142.0)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(180-157.0), Math.toRadians(20.0))
            .addParametricCallback(0.05) {follower.setMaxPower(1.0)}
            .build()

        val intakefirstspike = follower.pathBuilder().addPath(
            BezierLine(
                Pose(57.000, 73.000).mirror(142.0),

                Pose(19.000, 85.000).mirror(142.0)
            )
        ).setTangentHeadingInterpolation()

            .build()

        val shootfirstspike = follower.pathBuilder().addPath(
            BezierLine(
                Pose(19.000, 85.000).mirror(142.0),

                Pose(57.000, 73.000).mirror(142.0)
            )
        ).setTangentHeadingInterpolation()
            .setReversed()
            .build()

        val intakethirdspike = follower.pathBuilder().addPath(
            BezierCurve(
                Pose(57.000, 73.000).mirror(142.0),
                Pose(55.000, 32.000).mirror(142.0),
                Pose(45.000, 32.000).mirror(142.0),
                Pose(17.000, 34.000).mirror(142.0)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(18.0), Math.toRadians(0.0))

            .build()

        val shootthirdspike = follower.pathBuilder().addPath(
            BezierLine(
                Pose(17.000, 34.000).mirror(142.0),

                Pose(57.000, 78.000).mirror(142.0)
            )
        ).setTangentHeadingInterpolation()
            .setReversed()
            .build()

        val leave = follower.pathBuilder().addPath(
            BezierLine(
                Pose(57.000, 78.000).mirror(142.0),

                Pose(52.500, 72.500).mirror(142.0)
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
        Rollers.update()
        if(follower.pose != Pose(0.0,0.0,0.0)){
            ROBOT.teleopStartPose = follower.pose
        }
        telemetry.update()
    }
}