package org.firstinspires.ftc.teamcode.Autonomous

import com.pedropathing.geometry.BezierCurve
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
import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.currAlliance
import org.firstinspires.ftc.teamcode.Util.includePedro

// Made by Dan

@Autonomous(name = "Auto Red 12 12")
class Auto18Draft2Blue: NextFTCOpMode() {
    init {
        addSubsystems()
        includePedro (PedroConstants::createFollower)
    }
    private var Paths: Array<PathChain> = arrayOf()
    override fun onInit() {
        currAlliance = Alliance.BLUE
        follower.setStartingPose(
            Pose(111.0, 121.0, Math.toRadians(-90.0))
        )
        initiatePaths()

    }
    fun initiatePaths() {
        Paths += follower.pathBuilder().addPath(
            BezierLine(
                Pose(109.432, 137.104),

                Pose(111.000, 121.000)
            )
        ).setConstantHeadingInterpolation(Math.toRadians(-90.0))

            .build()

        Paths += follower.pathBuilder().addPath(
            BezierCurve(
                Pose(111.000, 121.000),
                Pose(112.051, 109.654),
                Pose(88.849, 84.772),
                Pose(104.306, 84.271)
            )
        ).setTangentHeadingInterpolation()

            .build()
    }
    override fun onStartButtonPressed() {
        val main = SequentialGroup (
            FollowPath(Paths[0]),
            Delay(0.25),
            FollowPath(Paths[1]),
        )
        main.schedule()
    }
}