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
import org.firstinspires.ftc.teamcode.Systems.Load.Load
import org.firstinspires.ftc.teamcode.Systems.Load.Rollers
import org.firstinspires.ftc.teamcode.Systems.Shooter.Shooter
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.includePedro
import org.firstinspires.ftc.teamcode.pedroPathing.Constants


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
            ParallelGroup(
                FollowPath(paths[2]), //shoot second spike mark
            ),
            Delay(0.2),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[3]), //intake gate
                InstantCommand { Rollers.run(1.0, 0.67) }
            ),
            Delay(1.5),
            ParallelGroup(FollowPath(paths[4]), //shoot gate
                //Delay(0.5),
                //InstantCommand { Rollers.run(0.0,0.0)}
            ),
            Delay(0.2),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[3]), //intake gate
                InstantCommand { Rollers.run(1.0, 0.67) }
            ),
            Delay(1.5),
            ParallelGroup(
                FollowPath(paths[4]), //shoot gate
                //Delay(0.5),
                //InstantCommand { Rollers.run(0.0,0.0)}
            ),
            Delay(0.2),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[3]), //intake gate
                InstantCommand { Rollers.run(1.0, 0.67) }
            ),
            Delay(1.5),
            ParallelGroup(
                FollowPath(paths[4]), //shoot gate
                //Delay(0.5),
                //InstantCommand { Rollers.run(0.0,0.0)}
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
            FollowPath(paths[7])
        )
        main.schedule()
    }
    fun buildPaths() {
        paths = arrayOf()

        val shootpreload = follower.pathBuilder().addPath(
            BezierLine(
                Pose(112.000, 130.750),

                Pose(87.000, 78.000)
            )
        ).setTangentHeadingInterpolation()
            .setReversed()
            .build()

        val intakespike2 = follower.pathBuilder().addPath(
            BezierCurve(
                Pose(87.000, 78.000),
                Pose(100.000, 58.000),
                Pose(130.000, 58.000)
            )
        ).setTangentHeadingInterpolation()

            .build()

        val shootspike2 = follower.pathBuilder().addPath(
            BezierLine(
                Pose(130.000, 58.000),

                Pose(87.000, 78.000)
            )
        ).setTangentHeadingInterpolation()
            .setReversed()
            .build()

        val intakegate = follower.pathBuilder().addPath(
            BezierLine(
                Pose(87.000, 78.000),
                Pose(108.350, 68.500)
            )
        ).setConstantHeadingInterpolation(Math.toRadians(0.0))
            .addPath(
                BezierLine(
                    Pose(108.350, 68.500),
                    Pose(129.700,59.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0.0),Math.toRadians(23.0))
            .build()

        /*val intakegate = follower.pathBuilder().addPath(
            BezierCurve(
                Pose(87.000, 78.000),
                Pose(90.000, 59.000),
                Pose(129.700, 59.000)
            )
        ).setConstantHeadingInterpolation(Math.toRadians(23.0))

            .build()*/

        val shootgate = follower.pathBuilder().addPath(
            BezierLine(
                Pose(129.700, 59.000),

                Pose(87.000, 78.000)
            )
        ).setTangentHeadingInterpolation()
            .setReversed()
            .build()

        val intakespike1 = follower.pathBuilder().addPath(
            BezierCurve(
                Pose(87.000, 78.000),
                Pose(105.500, 84.000),
                Pose(126.000, 84.000)
            )
        ).setTangentHeadingInterpolation()

            .build()

        val shootspike1 = follower.pathBuilder().addPath(
            BezierLine(
                Pose(126.000, 84.000),

                Pose(87.000, 78.000)
            )
        ).setTangentHeadingInterpolation()
            .setReversed()
            .build()

        val park = follower.pathBuilder().addPath(
            BezierLine(
                Pose(87.000, 78.000),

                Pose(128.000, 78.000)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(0.0))

            .build()
        paths += shootpreload
        paths += intakespike2
        paths += shootspike2
        paths += intakegate
        paths += shootgate
        paths += intakespike1
        paths += shootspike1
        paths += park
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