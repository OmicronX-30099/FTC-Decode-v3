package org.firstinspires.ftc.teamcode.Autonomous.Red.Collab

import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.Constants.PedroConstants
import org.firstinspires.ftc.teamcode.Systems.Load
import org.firstinspires.ftc.teamcode.Systems.Miscellaneous
import org.firstinspires.ftc.teamcode.Systems.Shooter
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.includePedro
import kotlin.math.PI

@Autonomous(name = "Leave auto Red", group = "Collab", preselectTeleOp = "Red TeleOp")
class LeaveAuto: NextFTCOpMode() {
    init {
        addSubsystems(Load, Miscellaneous, Shooter)
        includePedro(PedroConstants::createFollower)
    }

    lateinit var leavePath: PathChain

    override fun onInit() {
        ROBOT.currAlliance = Alliance.RED
        ROBOT.currStage = Stage.AUTONOMOUS
    }

    override fun onStartButtonPressed() {
        buildPaths()
        follower.setStartingPose(Pose(86.75,7.5,PI/2.0))
        val main =
            SequentialGroup(
                Delay(1.8),
                Load.shootTripleCommand,
                FollowPath(leavePath, true, 1.0)
            )
        main.schedule()
    }

    fun buildPaths() {
        leavePath = follower.pathBuilder()
            .addPath(
                BezierLine(
                    Pose(86.750, 7.5, Math.toRadians(90.0)), Pose(106.75, 7.5, Math.toRadians(90.0))
                )
            )
            .setConstantHeadingInterpolation(PI/2.0)
            .build()


    }

    override fun onUpdate() {
        Shooter.update()
        telemetry.update()
    }
    override fun onStop() { ROBOT.currTeleOpStartPose = follower.pose }
}