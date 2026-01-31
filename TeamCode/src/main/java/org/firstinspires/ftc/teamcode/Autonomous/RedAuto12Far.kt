package org.firstinspires.ftc.teamcode.Autonomous

import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
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
        follower.setStartingPose(Pose(70.500, 70.500, PI/2))
        buildPaths()
        val main = SequentialGroup(
            InstantCommand {
                IntakeSubsystem.intake(1.0)
                TransferSubsystem.transfer(0.35)
            },
            FollowPath(paths[0]),
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
            BezierLine(
                Pose(70.500, 70.500),

                Pose(70.500, 117.500)
            )
        ).setConstantHeadingInterpolation(Math.toRadians(90.0))

            .build()
        paths += Path1
    }
}