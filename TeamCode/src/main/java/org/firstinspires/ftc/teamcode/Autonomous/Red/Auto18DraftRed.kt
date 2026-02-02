package org.firstinspires.ftc.teamcode.Finals.AutoOpModes

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.ftc.components.LoopTimeComponent
import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.Systems.PassiveSystem
import org.firstinspires.ftc.teamcode.Systems.ShooterSystem
import org.firstinspires.ftc.teamcode.Util.currAlliance
import org.firstinspires.ftc.teamcode.Constants.PedroConstants
import org.firstinspires.ftc.teamcode.Util.currStartPose


@Autonomous(name = "18 Red Close Draft")
class Auto18DraftRed: NextFTCOpMode() {
    init {
        addComponents(
            BulkReadComponent,
            LoopTimeComponent(),
            PedroComponent(PedroConstants::createFollower),
            SubsystemComponent(ShooterSystem, PassiveSystem)
        )
    }
    var Paths: MutableMap<String, PathChain> = mutableMapOf()
    override fun onInit() {
        // Essentially sets the autoaim to track the blue goal.
        currAlliance = Alliance.RED
        // Sets where the Bot is positioned according to the code at the start
        follower.setStartingPose(
            Pose(
                PedroConstants.fieldWidth - 31,
                136.000,
                Math.toRadians(0.0)
            )
        )
        // Intiates the variables for the pathing
        this.buildPaths()
    }

    override fun onStartButtonPressed() {
        val main = SequentialGroup(
            FollowPath(Paths.getValue("ScoreLoad"), true, 0.7),
            Delay(0.25),
            FollowPath(Paths.getValue("IntakeOutOfOrder"), true, 0.7),
            Delay(0.25),
            FollowPath(Paths.getValue("PositionSecondSet"), true, 0.7),
            Delay(0.25),
            FollowPath(Paths.getValue("OpenGate1"), true, 0.7),
            Delay(0.25),
            FollowPath(Paths.getValue("MoveToFirstSet"), true, 0.7),
            Delay(0.25),
            FollowPath(Paths.getValue("PositionFirstSet"), true, 0.7),
            Delay(0.25),
            FollowPath(Paths.getValue("PositionGateBalls1"), true, 0.7),
            Delay(0.25),
            FollowPath(Paths.getValue("OpenGate2"), true, 0.7),
            Delay(0.25),
            FollowPath(Paths.getValue("PositionGateBalls2"), true, 0.7),
            Delay(0.25),
            FollowPath(Paths.getValue("MoveThirdSet"), true, 0.7),
            Delay(0.25),
            FollowPath(Paths.getValue("IntakeThirdSet"), true, 0.7),
            Delay(0.25),
            FollowPath(Paths.getValue("PositionThirdSet"), true, 0.7),

            )
        main.schedule()
    }

    // Intialize pathing variable, variable is of like a dictionary type from Python
    fun buildPaths() {
        // Function Meant to Intialize all the pathing variables
        // List of Paths:
        // "ScoreLoad", "IntakeOutOfOrder", "PositionFirstSet", "OpenGate1", "MoveToFirstSet" "PositionFirstSet"
        // "PositionGateBalls1" "OpenGate2" "PositionGateBalls2" "MoveThirdSet" "IntakeThirdSet" "PositionThirdSet"

        Paths.put("ScoreLoad", follower.pathBuilder().addPath(
            BezierLine(
                Pose(PedroConstants.fieldWidth-31, 136.000),

                Pose(PedroConstants.fieldWidth-58.750, 84.000)
            )
        ).setConstantHeadingInterpolation(Math.toRadians(0.0))

            .build())

        Paths.put("IntakeOutOfOrder", follower.pathBuilder().addPath(
            BezierCurve(
                Pose(PedroConstants.fieldWidth-58.750, 84.000),
                Pose(PedroConstants.fieldWidth-53.823, 56.980),
                Pose(PedroConstants.fieldWidth-8.000, 59.000)
            )
        ).setConstantHeadingInterpolation(Math.toRadians(0.0))

            .build())

        Paths.put("PositionSecondSet", follower.pathBuilder().addPath(
            BezierLine(
                Pose(PedroConstants.fieldWidth - 8.000, 59.000),

                Pose(PedroConstants.fieldWidth - 70.000, 73.000)
            )
        ).setConstantHeadingInterpolation(Math.toRadians(0.0))

            .build())

        Paths.put("OpenGate1", follower.pathBuilder().addPath(
            BezierLine(
                Pose(PedroConstants.fieldWidth - 70.000, 73.000),

                Pose(PedroConstants.fieldWidth - 10.000, 58.000)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(45.0))

            .build())
        Paths.put("MoveToFirstSet", follower.pathBuilder().addPath(
            BezierCurve(
                Pose(PedroConstants.fieldWidth - 10.000, 58.000),
                Pose(PedroConstants.fieldWidth - 42.226, 58.721),
                Pose(PedroConstants.fieldWidth - 58.000, 83.000)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(45.0), Math.toRadians(0.0))

            .build())

        Paths.put("PositionFirstSet", follower.pathBuilder().addPath(
            BezierLine(
                Pose(PedroConstants.fieldWidth - 58.000, 83.000),

                Pose(PedroConstants.fieldWidth - 14.000, 83.000)
            )
        ).setConstantHeadingInterpolation(Math.toRadians(0.0))

            .build())

        Paths.put("PositionGateBalls1", follower.pathBuilder().addPath(
            BezierLine(
                Pose(PedroConstants.fieldWidth - 14.000, 83.000),

                Pose(PedroConstants.fieldWidth - 70.000, 73.000)
            )
        ).setConstantHeadingInterpolation(Math.toRadians(0.0))

            .build())

        Paths.put("OpenGate2", follower.pathBuilder().addPath(
            BezierLine(
                Pose(PedroConstants.fieldWidth - 70.000, 73.000),

                Pose(PedroConstants.fieldWidth - 10.000, 58.000)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(45.0))

            .build())

        Paths.put("PositionGateBalls2", follower.pathBuilder().addPath(
            BezierLine(
                Pose(PedroConstants.fieldWidth - 10.000, 58.000),

                Pose(PedroConstants.fieldWidth - 34.000, 59.000)
            )
        ).setConstantHeadingInterpolation(Math.toRadians(45.0))

            .build())

        Paths.put("MoveThirdSet", follower.pathBuilder().addPath(
            BezierLine(
                Pose(PedroConstants.fieldWidth - 34.000, 59.000),

                Pose(PedroConstants.fieldWidth - 45.986, 35.224)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(45.0), Math.toRadians(0.0))

            .build())

        Paths.put("IntakeThirdSet", follower.pathBuilder().addPath(
            BezierLine(
                Pose(PedroConstants.fieldWidth - 45.986, 35.224),

                Pose(PedroConstants.fieldWidth - 8.266, 35.449)
            )
        ).setConstantHeadingInterpolation(Math.toRadians(0.0))

            .build())

        Paths.put("PositionThirdSet", follower.pathBuilder().addPath(
            BezierLine(
                Pose(PedroConstants.fieldWidth - 8.266, 35.449),

                Pose(PedroConstants.fieldWidth - 71.000, 20.000)
            )
        ).setConstantHeadingInterpolation(Math.toRadians(0.0))

            .build())
    }
    override fun onStop() {
        currStartPose = follower.pose //stores robot position for teleop usage
    }
}