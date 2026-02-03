package org.firstinspires.ftc.teamcode.Autonomous

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.Constants.PedroConstants
import org.firstinspires.ftc.teamcode.Enums.Alliance
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.BIMSubsystem
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.ShooterGateSubsystem
import org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems.TransferSubsystem
import org.firstinspires.ftc.teamcode.Systems.PassiveSystem
import org.firstinspires.ftc.teamcode.Systems.ShooterSystem
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.currAlliance
import org.firstinspires.ftc.teamcode.Util.includePedro
import kotlin.math.PI

@Autonomous(name = "Index test", group = "Test Autos")
class IndexTester: NextFTCOpMode() {
    init {
        addSubsystems(ShooterSystem, PassiveSystem)
        includePedro(PedroConstants::createFollower)
    }
    private var paths: Array<PathChain> = arrayOf()

    override fun onInit() {
        currAlliance = Alliance.RED
    }

    override fun onStartButtonPressed() {
        buildPaths()
        follower.setStartingPose(Pose(70.5,8.0,PI/2))
            /*ParallelGroup(
                FollowPath(paths[0],true,0.467),
                SequentialGroup(
                    InstantCommand { IntakeSubsystem.intake(1.0)
                        TransferSubsystem.transfer(0.1)},
                    Delay(0.5),
                    InstantCommand { BIMSubsystem.loadRight() },
                    Delay(1.5),
                    InstantCommand { IntakeSubsystem.intake(0.0) }
                )
            )*/
        /*val main = SequentialGroup(
                InstantCommand { BIMSubsystem.opengate() },
                InstantCommand {
                    TransferSubsystem.transfer(1.0)
                    ShooterGateSubsystem.open()
                },
                Delay(0.3),
                InstantCommand { BIMSubsystem.loadLeft() },
                Delay(0.4),
                InstantCommand { IntakeSubsystem.intake(1.0) },
                Delay(0.5),
                InstantCommand {
                    IntakeSubsystem.intake(0.0)
                    TransferSubsystem.transfer(0.0)
                    ShooterGateSubsystem.block()
                    BIMSubsystem.closeGate()
                }
        )*/
        /*val main = SequentialGroup(
            InstantCommand { BIMSubsystem.opengate() },
            InstantCommand {
                TransferSubsystem.transfer(1.0)
                ShooterGateSubsystem.open()
            },
            Delay(0.3),
            InstantCommand { BIMSubsystem.loadRight() },
            Delay(0.4),
            InstantCommand { IntakeSubsystem.intake(1.0) },
            Delay(0.5),
            InstantCommand {
                IntakeSubsystem.intake(0.0)
                TransferSubsystem.transfer(0.0)
                ShooterGateSubsystem.block()
                BIMSubsystem.closeGate()
            }
        )*/
        /*val main = SequentialGroup(
            InstantCommand {
                BIMSubsystem.opengate()
                BIMSubsystem.loadMiddle()
            },
            InstantCommand {
                TransferSubsystem.transfer(1.0)
                IntakeSubsystem.intake(1.0)
                ShooterGateSubsystem.open()
            },
            Delay(0.5),
            InstantCommand {
                BIMSubsystem.loadLeft() },
            Delay(0.4),
            InstantCommand {
                BIMSubsystem.loadRight() },
            Delay(0.5),
            InstantCommand {
                IntakeSubsystem.intake(0.0)
                TransferSubsystem.transfer(0.0)
                ShooterGateSubsystem.block()
                BIMSubsystem.closeGate()
            }


            )*/
        val main = SequentialGroup(
            FollowPath(paths[0]),
        )
        main.schedule()
    }

    override fun onUpdate() {
        ShooterSystem.updateShooter()
        telemetry.update()
    }

    /*fun buildPaths() {
        val startPose = Pose(78.5,8.0,Math.toRadians(90.0))
        val pushPose = Pose(90.0,8.0,Math.toRadians(90.0))
        val preloadPose = Pose(90.0,80.0, Math.toRadians(0.0))

        val firstIntakeControl = Pose(99.25, 49.75)
        val firstIntakePose = Pose(131.5,80.0,Math.toRadians(30.0))

        val firstShootControl = Pose(101.0,62.75)
        val firstShootPose = Pose(85.0,82.25,Math.toRadians(0.0))

        val secondIntakePose = Pose(127.5,82.25,Math.toRadians(0.0))

        val secondShootPose = Pose(81.5,82.25,Math.toRadians(0.0))

        val thirdIntakeControl = Pose(81.5,35.25)
        val thirdIntakePose = Pose(129.25,35.25,Math.toRadians(0.0))

        val stopPose = Pose(81.5,60.0,Math.toRadians(-90.0))

        val pushPath = follower.pathBuilder()
            .addPath(BezierLine(startPose, pushPose))
            .setConstantHeadingInterpolation(startPose.heading)
            .addPath(BezierLine(
                pushPose,
                preloadPose))
            .setLinearHeadingInterpolation(pushPose.heading, preloadPose.heading)
            .build()
        val firstIntake = follower.pathBuilder()
            .addPath(BezierCurve(
                preloadPose,
                firstIntakeControl,
                firstIntakePose
            ))
            .setLinearHeadingInterpolation(preloadPose.heading, firstIntakePose.heading)
            .addParametricCallback(
                0.5,
                { follower.setMaxPower(0.45) }
            )
            .addParametricCallback(
                1.0,
                { follower.setMaxPower(1.0) }
            )
            .build()
        val firstShoot = follower.pathBuilder()
            .addPath(BezierCurve(
                firstIntakePose,
                firstShootControl,
                firstShootPose
            ))
            .setLinearHeadingInterpolation(firstIntakePose.heading, firstShootPose.heading)
            .build()
        val secondIntake = follower.pathBuilder()
            .addPath(BezierLine(
                firstShootPose,
                secondIntakePose
            ))
            .setConstantHeadingInterpolation(secondIntakePose.heading)
            .addParametricCallback(
                0.5,
                { follower.setMaxPower(0.45) }
            )
            .addParametricCallback(
                1.0,
                { follower.setMaxPower(1.0) }
            )
            .build()
        val secondShoot = follower.pathBuilder()
            .addPath(BezierLine(
                secondIntakePose,
                secondShootPose
            ))
            .setConstantHeadingInterpolation(secondShootPose.heading)
            .build()
        val thirdIntake = follower.pathBuilder()
            .addPath(BezierCurve(
                secondShootPose,
                thirdIntakeControl,
                thirdIntakePose
            ))
            .setConstantHeadingInterpolation(thirdIntakePose.heading)
            .addParametricCallback(
                0.675,
                { follower.setMaxPower(0.45) }
            )
            .addParametricCallback(
                1.0,
                { follower.setMaxPower(1.0) }
            )
            .build()
        val thirdShoot = follower.pathBuilder()
            .addPath(BezierCurve(
                thirdIntakePose,
                thirdIntakeControl,
                secondShootPose
            ))
            .setLinearHeadingInterpolation(thirdIntakePose.heading, Math.toRadians(-90.0))
            .build()
        val leavePath = follower.pathBuilder()
            .addPath(BezierLine(
                secondShootPose,
                stopPose
            ))
            .setConstantHeadingInterpolation(stopPose.heading)
            .build()

        paths += pushPath
        paths += firstIntake
        paths += firstShoot
        paths += secondIntake
        paths += secondShoot
        paths += thirdIntake
        paths += thirdShoot
        paths += leavePath
    }*/
    fun buildPaths() {
        val test = follower.pathBuilder()
            .addPath(BezierLine(Pose(70.5,8.0),Pose(70.5,100.0)))
            .setConstantHeadingInterpolation(PI/2)
            .addParametricCallback(0.5,
                { follower.setMaxPower(0.4) }
            )
            .build()
    }
}