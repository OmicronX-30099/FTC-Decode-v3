package org.firstinspires.ftc.teamcode.Auto.Blue

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.ParallelRaceGroup
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

@Autonomous(name = "Blue Collab Auto", group = "Collab Auto", preselectTeleOp = "Blue TeleOp")
class BlueCollabFar(): NextFTCOpMode() {
    init {
        addSubsystems(Shooter, Load)
        includePedro(PedroConstants::createFollower)
    }
    private var paths: Array<PathChain> = arrayOf()

    override fun onInit() {
        ROBOT.currAlliance = Alliance.BLUE
        ROBOT.currStage = Stage.TELEOP
        Shooter.enableAutoAim()
    }
    override fun onStartButtonPressed() {
        buildPaths()
        follower.setStartingPose(Pose(86.750,7.5,Math.toRadians(90.0)).mirror())

        val main = SequentialGroup(
            Delay(1.5),
            Load.shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[0]),
                SequentialGroup (
                    InstantCommand { Rollers.run(1.0,0.25) }
                )
            ),
            InstantCommand { Rollers.stop() },
            FollowPath(paths[1]),
            Delay(0.5),
            Load.shootTripleCommand,
            ParallelRaceGroup(
                Delay(2.0),
                ParallelGroup(
                    FollowPath(paths[2]),//intake corner p1
                    InstantCommand { Rollers.run(1.0,0.25) }
                ),
            ),
            //FollowPath(paths[3]),//intake corner p2
            //FollowPath(paths[4]),//intake corner p3
            InstantCommand { Rollers.stop() },
            FollowPath(paths[5]),//shootcorner
            Delay(0.5),
            Load.shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[2]),//intake corner p1
                SequentialGroup(
                    InstantCommand { Rollers.run(1.0,0.25) }
                )
            ),
            Delay(0.15),
            InstantCommand { Rollers.stop() },
            FollowPath(paths[5]),//shoot corner
            Delay(0.5),
            Load.shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[2]),//intake corner p1
                SequentialGroup(
                    InstantCommand { Rollers.run(1.0,0.25) }
                )
            ),
            //FollowPath(paths[3]),//intake corner p2
            //FollowPath(paths[4]),//intake corner p3
            InstantCommand { Rollers.stop() },
            FollowPath(paths[5]),//shoot corner
            Delay(0.5),
            Load.shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[2]),//intake corner
                SequentialGroup(
                    InstantCommand { Rollers.run(1.0,0.25) }
                )
            ),
            Delay(0.15),
            InstantCommand { Rollers.stop() },
            FollowPath(paths[5]),//shoot corner
            Delay(0.5),
            Load.shootTripleCommand,
            FollowPath(paths[8],true,0.5)//leave
        )
        main.schedule()
    }

    override fun onUpdate() {
        Shooter.update()
        ROBOT.teleopStartPose = follower.pose
        telemetry.update()
    }
    fun buildPaths() {
        paths = arrayOf()
        val intakespike3 = follower.pathBuilder().addPath(
            BezierCurve(
                Pose(86.750, 7.500).mirror(),
                Pose(92.000, 34.000).mirror(),
                Pose(133.500, 35.000).mirror()
            )
        ).setLinearHeadingInterpolation(Math.toRadians(90.0), Math.toRadians(180.0))
            .build()
        val shootspike3 = follower.pathBuilder().addPath(
            BezierLine(
                Pose(130.000, 35.000).mirror(),
                Pose(90.000, 15.000).mirror()
            )
        ).setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
            .build()
        val intakecornerp1 = follower.pathBuilder().addPath(
            BezierLine(
                Pose(90.000, 15.000).mirror(),
                Pose(130.00, 10.000).mirror()
            )
        ).setTangentHeadingInterpolation()
            .build()
        val intakecornerp2 = follower.pathBuilder().addPath(
            BezierLine(
                Pose(130.000, 10.000).mirror(),
                Pose(126.500, 11.500).mirror()
            )
        ).setTangentHeadingInterpolation()
            .setReversed()
            .build()
        val intakecornerp3 = follower.pathBuilder().addPath(
            BezierLine(
                Pose(126.500, 11.500).mirror(),
                Pose(130.00, 10.000).mirror()
            )
        ).setTangentHeadingInterpolation()
            .build()
        val shootcorner = follower.pathBuilder().addPath(
            BezierLine(
                Pose(130.000, 10.000).mirror(),
                Pose(90.000, 15.000).mirror()
            )
        ).setTangentHeadingInterpolation()
            .setReversed()
            .build()
        val intaketunnel = follower.pathBuilder().addPath(
            BezierLine(
                Pose(90.000, 15.000).mirror(),
                Pose(131.500, 25.000).mirror()
            )
        ).setTangentHeadingInterpolation()
            .build()
        val shoottunnel = follower.pathBuilder().addPath(
            BezierLine(
                Pose(131.500, 25.000).mirror(),
                Pose(90.000, 15.000).mirror()
            )
        ).setTangentHeadingInterpolation()
            .setReversed()
            .build()
        val leave = follower.pathBuilder().addPath(
            BezierLine(
                Pose(90.000, 15.000).mirror(),
                Pose(100.000, 15.000).mirror()
            )
        ).setTangentHeadingInterpolation()
            .build()
        paths += intakespike3
        paths += shootspike3
        paths += intakecornerp1
        paths += intakecornerp2
        paths += intakecornerp3
        paths += shootcorner
        paths += intaketunnel
        paths += shoottunnel
        paths += leave
    }
}