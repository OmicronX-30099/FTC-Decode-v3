package org.firstinspires.ftc.teamcode.Auto.Red

import com.bylazar.gamepad.PanelsGamepad
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.instant
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.Constants.PedroConstants
import org.firstinspires.ftc.teamcode.Systems.Limelight
import org.firstinspires.ftc.teamcode.Systems.Load.BilinearIndexMachine
import org.firstinspires.ftc.teamcode.Systems.Load.Load
import org.firstinspires.ftc.teamcode.Systems.Load.Rollers
import org.firstinspires.ftc.teamcode.Systems.Shooter.Shooter
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.includePedro


@Autonomous(name = "Blob Tester", group = "Autos")
class SortedTester: NextFTCOpMode() {
    init {
        addSubsystems(Shooter, Load, Limelight)
        includePedro(PedroConstants::createFollower)
    }

    var paths: Array<PathChain> = arrayOf()
    var altpaths: Array<PathChain> = arrayOf()

    override fun onInit() {
        Shooter.reset()
        ROBOT.currStage = Stage.TELEOP
        ROBOT.currAlliance = Alliance.RED
    }

    override fun onStartButtonPressed() {
        Limelight.startBlobDetection()
        val main = SequentialGroup(
            Delay(5.0),
            FollowPath(
                (follower.pathBuilder()
                    .addPath(BezierLine(Limelight.getFollowPath().third, Limelight.getFollowPath().first))
                    .setConstantHeadingInterpolation(Limelight.getFollowPath().second)
                    .build())
            )
        )
        main.schedule()
    }

    override fun onUpdate() {
        val result = Limelight.getFollowPath()
        PanelsTelemetry.telemetry.addData("TargetPose", result.first)
        PanelsTelemetry.telemetry.addData("Heading", result.second)
        PanelsTelemetry.telemetry.addData("FollowerPose", result.third)
        PanelsTelemetry.telemetry.update(telemetry)
    }
}