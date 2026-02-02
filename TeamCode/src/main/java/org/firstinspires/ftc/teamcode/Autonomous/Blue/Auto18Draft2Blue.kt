package org.firstinspires.ftc.teamcode.Autonomous.Blue

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

@Autonomous(name = "")
class Auto18Draft2Blue: NextFTCOpMode() {
    init {
        addSubsystems()
        includePedro (PedroConstants::createFollower)
    }
    private var Paths: MutableMap<String, PathChain> = mutableMapOf()
    override fun onInit() {
        currAlliance = Alliance.BLUE
        follower.setStartingPose(
            Pose(63.135, 8.356, Math.toRadians(90.0))
        )
        initiatePaths()
        
    }
    fun initiatePaths() {
        Paths.put("ShootPre", follower.pathBuilder().addPath(
            BezierLine(
                Pose(63.135, 8.356),

                Pose(55.594, 8.428)
            )
        ).setConstantHeadingInterpolation(Math.toRadians(90.0))

        .build())

        Paths.put("GrabSecond", follower.pathBuilder().addPath(
            BezierCurve(
                Pose(55.594, 8.428),
                Pose(60.024, 62.517),
                Pose(10.305, 60.442)
            )
        ).setTangentHeadingInterpolation()

        .build())

        Paths.put("ShootSecond", follower.pathBuilder().addPath(
            BezierLine(
                Pose(10.305, 60.442),
        Pose(61.314, 82.370)
        )
        ).setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(200.0))

        .build())

        Paths.put("PositionAngle", follower.pathBuilder().addPath(
            BezierLine(
                    Pose(61.314, 82.370),

         Pose(42.773, 61.515)
        )
        ).setLinearHeadingInterpolation(Math.toRadians(200.0), Math.toRadians(180.0))

        .build())

        Paths.put("MoveToGateFollower", follower.pathBuilder().addPath(
            BezierLine(
                Pose(42.773, 61.515),

        Pose(20.131, 60.729)
        )
        ).setConstantHeadingInterpolation(Math.toRadians(180.0))

        .build())

        Paths.put("RotateOnGate", follower.pathBuilder().addPath(
            BezierLine(
                    Pose(20.131, 60.729),
                Pose(11.333, 61.149)
        )
        ).setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(135.0))

        .build())


        Paths.put("MoveToShoot", follower.pathBuilder().addPath(
            BezierLine(
                    Pose(11.333, 61.149),

        Pose(61.351, 82.694)
        )
        ).setConstantHeadingInterpolation(Math.toRadians(135.0))

        .build())

        Paths.put("RotateOnGate2", follower.pathBuilder().addPath(
            BezierLine(
                    Pose(61.351, 82.694),

        Pose(10.827, 60.694)
        )
        ).setConstantHeadingInterpolation(Math.toRadians(135.0))

        .build())

        Paths.put("MoveToShoot2", follower.pathBuilder().addPath(
            BezierLine(
                    Pose(10.827, 60.694),

        Pose(60.669, 82.199)
        )
        ).setConstantHeadingInterpolation(Math.toRadians(135.0))

        .build())

        Paths.put("RotateToGrabFirstSet", follower.pathBuilder().addPath(
            BezierLine(
                    Pose(60.669, 82.199),

        Pose(42.205, 82.184)
        )
        ).setLinearHeadingInterpolation(Math.toRadians(135.0), Math.toRadians(180.0))

        .build())

        Paths.put("GrabFirst", follower.pathBuilder().addPath(
            BezierLine(
                    Pose(42.205, 82.184),

        Pose(14.908, 81.942)
        )
        ).setTangentHeadingInterpolation()

        .build())

        Paths.put("MoveToShoot", follower.pathBuilder().addPath(
            BezierLine(
                    Pose(14.908, 81.942),

        Pose(62.274, 82.673)
        )
        ).setTangentHeadingInterpolation()
        .setReversed()
            .build());

        Paths.put("MoveToThird", follower.pathBuilder().addPath(
            BezierCurve(
                    Pose(62.274, 82.673),
        Pose(49.082, 84.749),
        Pose(62.930, 36.056),
        Pose(42.608, 36.212)
        )
        ).setTangentHeadingInterpolation()

        .build());

        Paths.put("GrabThird", follower.pathBuilder().addPath(
            BezierLine(
                    Pose(42.608, 36.212),

        Pose(7.462, 35.347)
        )
        ).setTangentHeadingInterpolation()

        .build())

        Paths.put("ShootThird", follower.pathBuilder().addPath(
            BezierLine(
                    Pose(7.462, 35.347),

        Pose(63.406, 9.162)
        )
        ).setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(135.0))

        .build())
    }
    override fun onStartButtonPressed() {
        val main = SequentialGroup (
            FollowPath(Paths.getValue("ShootPre")),
            Delay(0.25),
            FollowPath(Paths.getValue("ShootPre")),
            Delay(0.25),
            FollowPath(Paths.getValue("GrabSecond")),
            Delay(0.25),
            FollowPath(Paths.getValue("ShootSecond")),
            Delay(0.25),
            FollowPath(Paths.getValue("PositionAngle")),
            Delay(0.25),
            FollowPath(Paths.getValue("MoveToGateFollower")),
            Delay(0.25),
            FollowPath(Paths.getValue("RotateOnGate")),
            Delay(0.25),
            FollowPath(Paths.getValue("MoveToShoot")),
            Delay(0.25),
            FollowPath(Paths.getValue("RotateOnGate2")),
            Delay(0.25),
            FollowPath(Paths.getValue("MoveToShoot2")),
            Delay(0.25),
            FollowPath(Paths.getValue("MoveToThird")),
            Delay(0.25),
            FollowPath(Paths.getValue("GrabThird")),
            Delay(0.25),
            FollowPath(Paths.getValue("ShootThird")),
        )
        main.schedule()
    }
}