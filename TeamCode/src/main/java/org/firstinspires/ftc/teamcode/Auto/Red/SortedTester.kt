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


@Autonomous(name = "Sorted Tester", group = "Autos")
class SortedTester: NextFTCOpMode() {
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
        ParallelGroup(
            FollowPath(path),
            instant { Rollers.run(1.0,0.32); Rollers.lockShooter(); BilinearIndexMachine.toLeft() }
        ),
        Delay(initDelay),
        instant { BilinearIndexMachine.toRight(); Rollers.stop() },
        Delay(0.6),
        instant { Rollers.transfer(-0.35) },
        Delay(0.45),
        instant { Rollers.stop() },
        Delay(0.1),
        instant { BilinearIndexMachine.lockTransfer(); Rollers.run(1.0,0.5) },
        Delay(0.2),
        instant { Rollers.stop() }
    )

    override fun onStartButtonPressed() {
        follower.setStartingPose(Pose(0.0,0.0,Math.toRadians(90.0)))
        buildPaths()
        val main = SequentialGroup(
            sortedIntake(paths[0],0.3)
        )
        main.schedule()

    }

    fun buildPaths() {
        val test = follower.pathBuilder()
            .addPath(BezierLine(Pose(0.0,0.0),Pose(0.0, 24.0)))
            .setConstantHeadingInterpolation(Math.toRadians(90.0))
            .build()

        paths += test
    }

    override fun onUpdate() {
        Shooter.update()
        telemetry.update()
    }
}