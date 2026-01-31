package org.firstinspires.ftc.teamcode.Autonomous

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.Constants.PedroConstants
import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.Systems.PassiveSystem
import org.firstinspires.ftc.teamcode.Systems.ShooterSystem
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.currAlliance
import org.firstinspires.ftc.teamcode.Util.includePedro
import kotlin.math.PI


@Autonomous(name = "Red Far 12 ball Auto", group = "Red 12 autos", preselectTeleOp = "Red TeleOp - DP")
class PathTester: NextFTCOpMode() {
    init {
        addSubsystems(ShooterSystem, PassiveSystem)
        includePedro(PedroConstants::createFollower)
    }
    private var paths: Array<PathChain> = arrayOf()

    override fun onInit() {
        currAlliance = Alliance.RED
    }

    override fun onStartButtonPressed() {
        follower.setStartingPose(Pose(87.000, 9.700, PI/2))
        buildPaths()
        val main = SequentialGroup(
            FollowPath(paths[0],true, 1.0),
            FollowPath(paths[1],true, 1.0),
            FollowPath(paths[2],true, 1.0),
            FollowPath(paths[3],true, 1.0),
            FollowPath(paths[4],true, 1.0),
            FollowPath(paths[5],true, 1.0),
            FollowPath(paths[6],true, 1.0),
            //FollowPath(paths[7], true, 1.0),
        )
        main.schedule()
    }

    override fun onUpdate() {
        ShooterSystem.updateShooter()
        telemetry.update()
    }

    fun buildPaths() {
        val Path1 = follower
            .pathBuilder()
            .addPath(
                BezierCurve(
                    Pose(87.000, 9.700),
                    Pose(93.083, 36.796),
                    Pose(128.088, 35.602)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(90.0), Math.toRadians(0.0))
            .build()

        val Path2 = follower
            .pathBuilder()
            .addPath(
                BezierCurve(
                    Pose(128.088, 35.602),
                    Pose(102.033, 38.586),
                    Pose(88.309, 20.287)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(45.0))
            .build()

        val Path3 = follower
            .pathBuilder()
            .addPath(
                BezierCurve(
                    Pose(88.309, 20.287),
                    Pose(90.895, 59.271),
                    Pose(126.696, 59.072)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(45.0), Math.toRadians(0.0))
            .build()

        val Path4 = follower
            .pathBuilder()
            .addPath(
                BezierLine(Pose(126.696, 59.072), Pose(81.945, 84.928))
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()

        val Path5 = follower
            .pathBuilder()
            .addPath(
                BezierLine(Pose(81.945, 84.928), Pose(127.691, 84.331))
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()

        val Path6 = follower
            .pathBuilder()
            .addPath(
                BezierLine(Pose(127.691, 84.331), Pose(81.746, 85.127))
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()

        val Path7 = follower
            .pathBuilder()
            .addPath(
                BezierLine(Pose(81.746, 85.127), Pose(82.343, 51.514))
            )
            .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(179.0))
            .build()
        paths += Path1

        paths += Path2

        paths += Path3

        paths += Path4

        paths += Path5

        paths += Path6

        paths += Path7
    }
}