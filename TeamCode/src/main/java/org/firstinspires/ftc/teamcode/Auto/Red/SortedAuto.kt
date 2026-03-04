package org.firstinspires.ftc.teamcode.Auto.Red

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.instant
import dev.nextftc.core.commands.utility.NullCommand
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.Constants.PedroConstants
import org.firstinspires.ftc.teamcode.Systems.Load.BilinearIndexMachine
import org.firstinspires.ftc.teamcode.Systems.Load.Load
import org.firstinspires.ftc.teamcode.Systems.Load.Rollers
import org.firstinspires.ftc.teamcode.Systems.Shooter.Shooter
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.includePedro


@Autonomous(name = "Sorted Red Auto", group = "Autos", preselectTeleOp = "Red TeleOp")
class SortedAuto: NextFTCOpMode() {
    init {
        addSubsystems(Shooter, Load)
        includePedro(PedroConstants::createFollower)
    }

    var paths: Array<PathChain> = arrayOf()
    var altpaths: Array<PathChain> = arrayOf()

    override fun onInit() {
        Shooter.reset()
        ROBOT.currStage = Stage.TELEOP
        ROBOT.currAlliance = Alliance.RED
    }


    val LMR: Command = SequentialGroup(
        instant { Rollers.unlockShooter(); BilinearIndexMachine.unlockTransfer(); Rollers.run(1.0,0.65) },
        Delay(0.8),
        instant { BilinearIndexMachine.toLeft() },
        Delay(0.5),
        instant {
            Rollers.stop()
            Rollers.lockShooter()
        }
    )
    val RLM: Command = SequentialGroup(
        instant { Rollers.unlockShooter(); BilinearIndexMachine.unlockTransfer(); Rollers.transfer(0.7); BilinearIndexMachine.toLeft() },
        Delay(0.4),
        instant { BilinearIndexMachine.toRight() },
        Delay(0.7),
        instant { Rollers.intake(1.0) },
        Delay(0.9),
        instant { Rollers.stop() }
    )
    val MRL: Command = SequentialGroup(
        instant { Rollers.unlockShooter(); BilinearIndexMachine.unlockTransfer(); Rollers.run(1.0,0.7); BilinearIndexMachine.split() },
        Delay(0.8),
        instant { BilinearIndexMachine.toLeft() },
        Delay(0.6),
        instant { BilinearIndexMachine.toRight() },
        Delay(0.7),
        instant { Rollers.stop() }
    )

    fun straightIntake(path: PathChain) = SequentialGroup(
        ParallelGroup(
            FollowPath(path),
            instant { Rollers.run(1.0,0.32); Rollers.lockShooter(); BilinearIndexMachine.toLeft() }
        ),
        instant { Rollers.stop() }
    )

    fun sortedIntake(path: PathChain, endDelay: Double, extra: Command = NullCommand(), ) = SequentialGroup(
        straightIntake(path),
        Delay(endDelay),
        SequentialGroup(
            instant { BilinearIndexMachine.toRight() },
            Delay(0.6),
            instant { Rollers.transfer(-0.35) },
            Delay(0.45),
            instant { Rollers.stop() },
            Delay(0.1),
            instant { BilinearIndexMachine.lockTransfer(); Rollers.run(1.0,0.5) },
            Delay(0.2),
            instant { Rollers.stop() },
            extra
        ).asProxy()
    )


    override fun onStartButtonPressed() {
        Shooter.reset()
        follower.setStartingPose(Pose(79.750, 7.500,Math.toRadians(90.0)))
        buildPaths()
        Shooter.flywheelManual()
        Shooter.increaseFlywheelVel(1460.0)
        val pgp = SequentialGroup(
            FollowPath(paths[0]),
            Delay(0.4),
            Load.shootTripleCommand,
            sortedIntake(paths[1],0.1),
            FollowPath(paths[2]),
            Delay(0.5),
            FollowPath(paths[3]),
            Delay(0.4),
            LMR,
            Delay(0.2),
            sortedIntake(paths[4], 0.2, instant { BilinearIndexMachine.toLeft() }),
            FollowPath(paths[5]),
            Delay(0.3),
            RLM,
            straightIntake(paths[6]),
            Delay(0.15),
            FollowPath(paths[7]),
            Delay(0.3),
            Load.shootCommand(1.0,0.4,1.0),
            Delay(0.15),
            FollowPath(paths[8])
        )
        pgp.schedule()

        val gpp = SequentialGroup(
            FollowPath(paths[0]),
            Delay(0.4),
            Load.shootTripleCommand,
            sortedIntake(paths[1],0.1, instant { BilinearIndexMachine.split() }),
            FollowPath(paths[2]),
            Delay(0.5),
            FollowPath(paths[3]),
            Delay(0.4),
            MRL,
            Delay(0.2),
            straightIntake(paths[4]),
            FollowPath(paths[5]),
            Delay(0.3),
            Load.shootCommand(1.0,0.4,1.0),
            sortedIntake(paths[6], 0.15, instant { BilinearIndexMachine.toLeft() }),
            FollowPath(paths[7]),
            Delay(0.3),
            RLM,
            Delay(0.15),
            FollowPath(paths[8])
        )

    }

    fun buildPaths() {
        val shootPreload = follower.pathBuilder()
            .addPath(BezierCurve(Pose(79.750, 7.500),Pose(100.000, 6.000),Pose(82.750, 82.250)))
            .setConstantHeadingInterpolation(Math.toRadians(90.0))
            .build()
        val intakeSpike1 = follower.pathBuilder()
            .addPath(BezierLine(Pose(82.750, 82.250),Pose(124.750, 82.250)))
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val openGate = follower.pathBuilder()
            .addPath(BezierCurve(Pose(124.750, 82.250),Pose(120.000, 79.375),Pose(124.250, 76.500)))
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val shootSet1 = follower.pathBuilder()
            .addPath(BezierLine(Pose(124.250, 77.250),Pose(82.750, 82.250)))
            .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(-10.0))
            .build()
        val intakeSpike2 = follower.pathBuilder()
            .addPath(BezierLine(Pose(82.750, 82.250),Pose(100.750, 35.250)))
            .setLinearHeadingInterpolation(Math.toRadians(-70.0), Math.toRadians(0.0))
            .addPath(BezierLine(Pose(100.750, 35.250),Pose(121.750, 35.250)))
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val shootSet2 = follower.pathBuilder()
            .addPath(BezierCurve(Pose(121.750, 35.250),Pose(94.500, 53.000),Pose(82.750, 82.250)))
            .setTangentHeadingInterpolation()
            .setReversed()
            .build()
        val IntakeSpike3 = follower.pathBuilder()
            .addPath(BezierLine(Pose(82.750, 82.250),Pose(100.750, 58.750)))
            .setLinearHeadingInterpolation(Math.toRadians(-53.0), Math.toRadians(0.0))
            .addPath(BezierLine(Pose(100.750, 58.750),Pose(121.750, 58.750)))
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val shootSet3 = follower.pathBuilder()
            .addPath(BezierLine(Pose(121.750, 58.750),Pose(82.750, 82.250)))
            .setTangentHeadingInterpolation()
            .setReversed()
            .build()
        val leave = follower.pathBuilder()
            .addPath(BezierLine(Pose(82.750, 82.250),Pose(118.000, 66.600)))
            .setConstantHeadingInterpolation(Math.toRadians(-31.0))
            .build()

        paths += shootPreload
        paths += intakeSpike1
        paths += openGate
        paths += shootSet1
        paths += intakeSpike2
        paths += shootSet2
        paths += IntakeSpike3
        paths += shootSet3
        paths += leave
    }

    fun buildAlt() {
        val preload = follower.pathBuilder().addPath(
            BezierLine(
                Pose(34.000, 132.900).mirror(),

                Pose(23.5*4.5-23.0, 23.5*3.5)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(0.0))

            .build()
        val intake1 = follower.pathBuilder()
            .addPath(BezierLine(Pose(57.0, 73.0).mirror(), Pose(23.5*4.5-5.0, 23.5*3.5)))
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .addPath(BezierLine(Pose(23.5*4.5-2.0, 23.5*3.5),Pose(23.5*4.5+16.0,23.5*3.5)))
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val shoot1 = follower.pathBuilder()
            .addPath(BezierLine(Pose(23.5*4.5+16.0,23.5*3.5), Pose(23.5*4.5-23.0, 23.5*3.5)))
            .setConstantHeadingInterpolation(0.0)
            .build()
        altpaths += preload
        altpaths += intake1
        altpaths += shoot1
    }

    override fun onUpdate() {
        Shooter.update()
        telemetry.update()
    }
}