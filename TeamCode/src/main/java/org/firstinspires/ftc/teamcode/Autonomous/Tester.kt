package org.firstinspires.ftc.teamcode.Autonomous

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.Constants.PedroConstants
import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.TransferSubsystem
import org.firstinspires.ftc.teamcode.Systems.PassiveSystem
import org.firstinspires.ftc.teamcode.Systems.ShooterSystem
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.currAlliance
import org.firstinspires.ftc.teamcode.Util.includePedro
import kotlin.math.PI


@Autonomous(name = "Tester", group = "Tests")
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
            InstantCommand {
                IntakeSubsystem.intake(1.0)
                TransferSubsystem.transfer(0.35)
            },
            FollowPath(paths[0]),
            ParallelGroup(
                FollowPath(paths[1]),
                SequentialGroup(
                    Delay(1.0),
                    InstantCommand {
                        IntakeSubsystem.intake(0.0)
                        TransferSubsystem.transfer(0.0)
                    }
                )
            ),
            PassiveSystem.shootTripleCommand
        )
        main.schedule()
    }

    override fun onUpdate() {
        ShooterSystem.updateShooter()
        telemetry.update()
    }

    fun buildPaths() {
        val Path1 = follower.pathBuilder().addPath(
            BezierCurve(
                Pose(87.000, 9.700),
                Pose(95.500, 33.000),
                Pose(130.000, 35.000)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(90.0), Math.toRadians(0.0))

            .build()
        val Path2 = follower.pathBuilder().addPath(
            BezierCurve(
                Pose(130.000, 35.000),
                Pose(98.500, 36.000),
                Pose(81.100, 14.500)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(45.0))

            .build()
        paths += Path1
        paths += Path2
    }
}