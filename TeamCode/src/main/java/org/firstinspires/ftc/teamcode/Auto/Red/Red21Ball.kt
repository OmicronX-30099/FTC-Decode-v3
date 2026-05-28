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
import org.firstinspires.ftc.teamcode.Systems.Load.BreakBeam
import org.firstinspires.ftc.teamcode.Systems.Load.Load
import org.firstinspires.ftc.teamcode.Systems.Load.Rollers
import org.firstinspires.ftc.teamcode.Systems.Shooter.Shooter
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.includePedro
import org.firstinspires.ftc.teamcode.pedroPathing.Constants


@Autonomous(name = "Red 21 Ball", group = "Collab Auto", preselectTeleOp = "Red TeleOp")
class Red21Ball(): NextFTCOpMode() {
    init {
        addSubsystems(Load, Shooter)
        includePedro(Constants::createFollower)
    }

    private lateinit var paths: Paths

    override fun onInit() {
        ROBOT.currAlliance = Alliance.RED
        Shooter.reset()
        ROBOT.currStage = Stage.AUTONOMOUS
        follower.poseTracker.resetIMU()
    }
    override fun onStartButtonPressed() {
        follower.setStartingPose(Pose(112.0,130.75,Math.toRadians(90.0)))
        buildPaths()
        Shooter.flywheelManual(1420.0)
        val main = SequentialGroup(
            FollowPath(paths.shootpreload),
            //Delay(0.2),
            Load.shootTripleCommand,
            ParallelGroup(
                FollowPath(paths.fullspike2),
                InstantCommand { Rollers.run(1.0,0.2) }
            ),
            //Delay(0.2),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths.intakegate),
                InstantCommand { Rollers.run(1.0, 0.2) }
            ),
            Delay(1.0),
            ParallelGroup(FollowPath(paths.shootgate),
                //Delay(0.5),
                //InstantCommand { Rollers.run(0.0,0.0)}
            ),
            //Delay(0.2),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths.intakegate),
                InstantCommand { Rollers.run(1.0, 0.2) }
            ),
            Delay(1.5),
            ParallelGroup(
                FollowPath(paths.shootgate),
                //Delay(0.5),
                //InstantCommand { Rollers.run(0.0,0.0)}
            ),
            //Delay(0.2),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths.intakegate),
                InstantCommand { Rollers.run(1.0, 0.2) }
            ),
            Delay(1.5),
            ParallelGroup(
                FollowPath(paths.shootgate),
                //Delay(0.5),
                //InstantCommand { Rollers.run(0.0,0.0)}
            ),
            //Delay(0.2),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths.intakegate),
                InstantCommand { Rollers.run(1.0, 0.2) }
            ),
            Delay(1.5),
            ParallelGroup(
                FollowPath(paths.shootgate),
                //Delay(0.5),
                //InstantCommand { Rollers.run(0.0,0.0)}
            ),
            //Delay(0.2),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths.fullspike1),
                InstantCommand { Rollers.run(1.0, 0.2) }
            ),
            //Delay(0.2),
            Load . shootTripleCommand,
            FollowPath(paths.park)
        )
        main.schedule()
    }
    fun buildPaths() {
        paths = Paths()
    }

    inner class Paths {
        val shootpreload: PathChain = follower.pathBuilder().addPath(
            BezierLine(
                Pose(112.000, 130.750),

                Pose(87.000, 78.000)
            )
        ).setTangentHeadingInterpolation()
            .setReversed()
            .build()

        val fullspike2: PathChain = follower.pathBuilder().addPath(
            BezierCurve(
                Pose(87.000, 78.000),
                Pose(100.000, 58.000),
                Pose(130.000, 58.000)
            )
        ).setTangentHeadingInterpolation()
            .addPath(
                BezierLine(
                    Pose(130.000, 58.000),
                    Pose(87.000, 78.000)
                )
            ).setTangentHeadingInterpolation()
            .setReversed()
            .build()

        val intakegate: PathChain = follower.pathBuilder().addPath(
            BezierLine(
                Pose(87.000, 78.000),
                Pose(108.350, 68.500)
            )
        ).setConstantHeadingInterpolation(Math.toRadians(0.0))
            .addPath(
                BezierLine(
                    Pose(108.350, 68.500),
                    Pose(130.700,59.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0.0),Math.toRadians(27.0))
            .build()

        /*val intakegate = follower.pathBuilder().addPath(
            BezierCurve(
                Pose(87.000, 78.000),
                Pose(90.000, 59.000),
                Pose(129.700, 59.000)
            )
        ).setConstantHeadingInterpolation(Math.toRadians(23.0))

            .build()*/

        val shootgate: PathChain = follower.pathBuilder().addPath(
            BezierLine(
                Pose(130.700, 59.000),

                Pose(87.000, 78.000)
            )
        ).setTangentHeadingInterpolation()
            .setReversed()
            .build()

        val fullspike1: PathChain = follower.pathBuilder().addPath(
            BezierCurve(
                Pose(87.000, 78.000),
                Pose(105.500, 80.000),
                Pose(126.000, 80.000)
            )
        ).setTangentHeadingInterpolation()
            .addPath(
                BezierLine(
                    Pose(126.000, 80.000),
                    Pose(87.000, 78.000)
                )
            ).setTangentHeadingInterpolation()
            .setReversed()
            .build()

        val park: PathChain = follower.pathBuilder().addPath(
            BezierLine(
                Pose(87.000, 78.000),

                Pose(125.5,78.000)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(0.0))

            .build()
    }
    override fun onUpdate() {
        val currentBallCount = BreakBeam.refreshBallCount(minIntervalMs = 20L)
        Shooter.update()
        Rollers.update(currentBallCount)
        if(follower.pose != Pose(0.0,0.0,0.0)){
            ROBOT.teleopStartPose = follower.pose
        }
        telemetry.update()
    }
}
