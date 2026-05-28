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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.Systems.Load.Load
import org.firstinspires.ftc.teamcode.Systems.Load.Rollers
import org.firstinspires.ftc.teamcode.Systems.Shooter.Shooter
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.includePedro

 @Autonomous(name = "Red Collab Auto", group = "Collab Auto", preselectTeleOp = "Red TeleOp")
class RedCollabFar(): NextFTCOpMode() {
    init {
        addSubsystems(Shooter, Load)
        includePedro(Constants::createFollower)
    }
    private lateinit var paths: Paths

    override fun onInit() {
        ROBOT.currAlliance = Alliance.RED
        ROBOT.currStage = Stage.TELEOP
        Shooter.enableAutoAim()
        follower.poseTracker.resetIMU()
    }
    override fun onStartButtonPressed() {
        buildPaths()
        follower.setStartingPose(Pose(86.112, 6.250, Math.toRadians(0.0)))
        Shooter.flywheelManual(1740.0)

        fun fullswipeCommand() = SequentialGroup(
            ParallelGroup(
                FollowPath(paths.fullswipe),
                InstantCommand { Rollers.run(1.0,0.2) }
            ),
            InstantCommand { Rollers.stop() },
            Delay(0.2),
            Load.shootTripleCommand
        )

        val main = SequentialGroup(
            FollowPath(paths.shootpreload),
            Delay(0.8),
            Load.shootTripleCommand,
            ParallelGroup(
                FollowPath(paths.full3rdspike),
                InstantCommand { Rollers.run(1.0,0.2) }
            ),
            InstantCommand { Rollers.stop() },
            Delay(0.2),
            Load.shootTripleCommand,
            ParallelGroup(
                FollowPath(paths.intakecorner),
                InstantCommand { Rollers.run(1.0,0.2) }
            ),
            Delay(0.5),
            InstantCommand { Rollers.stop() },
            FollowPath(paths.shootercorner),
            Delay(0.2),
            Load.shootTripleCommand,
            fullswipeCommand(),
            fullswipeCommand(),
            fullswipeCommand(),
            fullswipeCommand()
        )
        main.schedule()
    }

    override fun onUpdate() {
        val currentBallCount = BreakBeam.refreshBallCount(minIntervalMs = 20L)
        Shooter.update()
        Rollers.update(currentBallCount)
        ROBOT.teleopStartPose = follower.pose
        telemetry.update()
    }
    fun buildPaths() {
        paths = Paths()
    }

    inner class Paths {
        val shootpreload: PathChain = follower.pathBuilder().addPath(
            BezierLine(
                Pose(86.112, 6.250),
                Pose(86.112, 19.000)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(0.0))
            .build()

        val full3rdspike: PathChain = follower.pathBuilder().addPath(
            BezierCurve(
                    Pose(86.112, 19.000),
                Pose(86.112, 35.000),
                Pose(125.000, 35.000)
        )
        ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(0.0))
            .addPath(
                BezierLine(
                    Pose(125.000, 35.000),
                    Pose(94.000, 9.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(0.0))
            .build()

        val intakecorner: PathChain = follower.pathBuilder().addPath(
            BezierLine(
                Pose(94.000, 9.000),
                Pose(131.000, 8.000)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(0.0))
            .build()

        val shootercorner: PathChain = follower.pathBuilder().addPath(
            BezierLine(
                Pose(131.000, 8.000),
                Pose(94.000, 9.000)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(0.0))
            .build()

        val fullswipe: PathChain = follower.pathBuilder().addPath(
            BezierLine(
                Pose(94.000, 9.000),
                Pose(125.000, 9.000)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(0.0))
            .addPath(
                BezierCurve(
                    Pose(125.000, 9.000),
                    Pose(132.000, 9.000),
                    Pose(132.000, 35.000)
                )
            ).setTangentHeadingInterpolation()
            .addPath(
                BezierLine(
                    Pose(132.000, 35.000),
                    Pose(94.000, 9.000)
                )
            ).setTangentHeadingInterpolation()
            .setReversed()
            .build()
    }
}
