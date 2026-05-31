package org.firstinspires.ftc.teamcode.Auto.Red

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.ParallelRaceGroup
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


@Autonomous(name = "Blue Close No Gate", group = "Collab Auto", preselectTeleOp = "Blue TeleOp")
class BlueCloseNoGate(): NextFTCOpMode() {
    init {
        addSubsystems(Load, Shooter)
        includePedro(Constants::createFollower)
    }

    private lateinit var paths: Paths

    override fun onInit() {
        ROBOT.currAlliance = Alliance.BLUE
        Shooter.reset()
        ROBOT.currStage = Stage.AUTONOMOUS
        follower.poseTracker.resetIMU()
    }
    override fun onStartButtonPressed() {
        follower.setStartingPose(Pose(112.0,130.75,Math.toRadians(90.0)).mirror())
        buildPaths()
        Shooter.flywheelManual(1420.0)

        fun threeBallsOrTimeout() = ParallelRaceGroup(
            WaitUntil { BreakBeam.refreshBallCount(minIntervalMs = 20L) >= 3 },
            Delay(1.5)
        )

        val main = SequentialGroup(
            FollowPath(paths.preload),
            Load.shootTripleCommand,
            ParallelGroup(
                FollowPath(paths.pathChain22ndspike),
                InstantCommand { Rollers.run(1.0,0.25) }
            ),
            Load.shootTripleCommand,
            ParallelGroup(
                FollowPath(paths.intake1stspike),
                InstantCommand { Rollers.run(1.0, 0.25) }
            ),
            InstantCommand { Rollers.stop() },
            FollowPath(paths.gate),
            Delay(1.0),
            FollowPath(paths.shoot1stspike),
            Load.shootTripleCommand,
            ParallelGroup(
                FollowPath(paths.pathChain63rdspike),
                InstantCommand { Rollers.run(1.0,0.25) }
            ),
            Load.shootTripleCommand,
            ParallelGroup(
                FollowPath(paths.corner),
                InstantCommand { Rollers.run(1.0, 0.25) }
            ),
            Load.shootTripleCommand,
            ParallelGroup(
                FollowPath(paths.corner),
                InstantCommand { Rollers.run(1.0, 0.25) }
            ),
            Load.shootTripleCommand,
            FollowPath(paths.leave)
        )
        main.schedule()
    }
    fun buildPaths() {
        paths = Paths()
    }

    inner class Paths {
        val preload = follower.pathBuilder()
            .addPath(
                BezierLine(
                    Pose(112.000, 130.750).mirror(),
                    Pose(87.000, 78.000).mirror()
                )
            )
            .setTangentHeadingInterpolation()
            .setReversed()
            .build()

        val pathChain22ndspike = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Pose(87.000, 78.000).mirror(),
                    Pose(100.000, 58.000).mirror(),
                    Pose(130.000, 58.000).mirror()
                )
            )
            .setTangentHeadingInterpolation()
            .addPath(
                BezierLine(
                    Pose(130.000, 58.000).mirror(),
                    Pose(87.000, 78.000).mirror()
                )
            )
            .setTangentHeadingInterpolation()
            .setReversed()
            .build();

        val intake1stspike = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Pose(87.000, 78.000).mirror(),
                    Pose(105.500, 83.000).mirror(),
                    Pose(120.000, 83.000).mirror()
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
            .build();

        val gate = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Pose(120.000, 83.000).mirror(),
                    Pose(120.000, 70.000).mirror(),
                    Pose(125.000, 70.000).mirror()
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build();

        val shoot1stspike = follower.pathBuilder()
            .addPath(
                BezierLine(
                    Pose(124.000, 70.000).mirror(),
                    Pose(87.000, 78.000).mirror()
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build();

        val pathChain63rdspike = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Pose(87.000, 78.000).mirror(),
                    Pose(87.000, 35.000).mirror(),
                    Pose(120.000, 35.000).mirror()
                )
            )
            .setTangentHeadingInterpolation()
            .addPath(
                BezierLine(
                    Pose(120.000, 35.000).mirror(),
                    Pose(87.000, 78.000).mirror()
                )
            )
            .setTangentHeadingInterpolation()
            .setReversed()
            .build();

        val corner = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Pose(87.000, 78.000).mirror(),
                    Pose(130.000, 45.000).mirror(),
                    Pose(130.000, 12.000).mirror()
                )
            )
            .setTangentHeadingInterpolation()
            .addPath(
                BezierLine(
                    Pose(130.000, 12.000).mirror(),
                    Pose(87.000, 78.000).mirror()
                )
            )
            .setTangentHeadingInterpolation()
            .setReversed()
            .build();

        val leave = follower.pathBuilder()
            .addPath(
                BezierLine(
                    Pose(87.000, 78.000).mirror(),
                    Pose(125.500, 78.000).mirror()
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build();
    }
    override fun onUpdate() {
        val currentBallCount = BreakBeam.refreshBallCount(minIntervalMs = 20L)
        Shooter.update()
        Rollers.update(currentBallCount)
        if(follower.pose != Pose(0.0,0.0,0.0).mirror()){
            ROBOT.blueTeleopStartPose = follower.pose
        }
        telemetry.update()
    }
}
