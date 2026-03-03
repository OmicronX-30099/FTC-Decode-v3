package org.firstinspires.ftc.teamcode.Auto.Red

import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.instant
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
import kotlin.math.PI

@Autonomous(name = "Sorted Red Auto", group = "Autos", preselectTeleOp = "Red TeleOp")
class SortedAuto: NextFTCOpMode() {
    init {
        addSubsystems(Shooter, Load)
        includePedro(PedroConstants::createFollower)
    }

    var paths: Array<PathChain> = arrayOf()

    override fun onInit() {
        Shooter.reset()
        ROBOT.currStage = Stage.TELEOP
        ROBOT.currAlliance = Alliance.RED
    }


    val LMR: Command = SequentialGroup(
        instant { Rollers.unlockShooter(); BilinearIndexMachine.unlockTransfer(); Rollers.run(0.6,0.6) },
        Delay(1.0),
        instant { BilinearIndexMachine.toLeft() },
        Delay(0.5),
        instant {
            Rollers.stop()
            Rollers.lockShooter()
        }
    )

    fun sortedIntake(path: PathChain, initDelay: Double) = SequentialGroup(
        Delay(initDelay),
        ParallelGroup(
            FollowPath(path),
            instant {
                Rollers.intake(0.55)
                Rollers.lockShooter()
                BilinearIndexMachine.lockTransfer()
                BilinearIndexMachine.toLeft()
            }
        ),
        instant { BilinearIndexMachine.toRight(); Rollers.stop() },
        Delay(0.6),
        instant { Rollers.intake(0.7,) },
        Delay(0.5),
        instant { Rollers.stop() }
    )

    override fun onStartButtonPressed() {
        follower.setStartingPose(Pose(34.0,132.9,Math.toRadians(-180.0)).mirror())
        buildPaths()
        val main = SequentialGroup(
            FollowPath(paths[0]),
            Delay(0.4),
            Load.shootTripleCommand,
            sortedIntake(paths[1],0.6),
            Delay(0.1),
            FollowPath(paths[2]),
            LMR
        )
        main.schedule()

    }

    fun buildPaths() {
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
        paths += preload
        paths += intake1
        paths += shoot1
    }

    override fun onUpdate() {
        Shooter.update()
        telemetry.update()
    }
}