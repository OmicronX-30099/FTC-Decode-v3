package org.firstinspires.ftc.teamcode.Autonomoous.Red

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
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
import org.firstinspires.ftc.teamcode.Systems.Load
import org.firstinspires.ftc.teamcode.Systems.LoadSubsystems.Rollers
import org.firstinspires.ftc.teamcode.Systems.Miscellaneous
import org.firstinspires.ftc.teamcode.Systems.Shooter
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.Flywheel
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.FlywheelState
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.includePedro

@Autonomous(name = "Unsorted 15-ball Blue", group = "Unsorted Auto", preselectTeleOp = "Blue TeleOp")
class Unsorted15AutoBlue(): NextFTCOpMode() {
    init {
        addSubsystems(Load, Shooter, Miscellaneous)
        includePedro(PedroConstants::createFollower)
    }

    private var paths: Array<PathChain> = arrayOf()

    override fun onInit() {
        ROBOT.currAlliance = Alliance.BLUE
        ROBOT.currStage = Stage.AUTONOMOUS
        ROBOT.currStage.useFlywheelVel = false
    }
    override fun onStartButtonPressed() {
        follower.setStartingPose(Pose(34.0,132.9,Math.toRadians(-180.0)))
        buildPaths()
        val main = SequentialGroup(
            InstantCommand { Shooter.setFlywheelManualVelocity(Shooter.getVelocity(Pose(55.0,73.0)))},
            FollowPath(paths[0]), //shoot preload
            Delay(0.6),
            Load.shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[1]), //intake second spike mark
                InstantCommand { Rollers.run(0.3, 1.0) }
            ),
            InstantCommand { Shooter.setFlywheelManualVelocity(Shooter.getVelocity(Pose(55.0,73.0)))},
            ParallelGroup(FollowPath(paths[2]), //shoot second spike mark
                Delay(1.0),
                InstantCommand { Rollers.run(0.0,0.0)},
                ),
            Delay(0.6),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[3],true,1.0), //intake gate
                InstantCommand { Rollers.run(0.3, 1.0) }
            ),
            Delay(0.5),
            InstantCommand { Shooter.setFlywheelManualVelocity(Shooter.getVelocity(Pose(55.0,73.0)))},
            ParallelGroup(FollowPath(paths[4],true,1.0), //shoot gate
                InstantCommand { Delay(1.0) },
                InstantCommand { Rollers.run(0.0,0.0)},
            ),
            Delay(0.6),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[5]), //intake first spike
                InstantCommand { Rollers.run(0.3, 1.0) }
            ),
            InstantCommand { Rollers.run(0.0,0.0)},
            InstantCommand { Shooter.setFlywheelManualVelocity(Shooter.getVelocity(Pose(55.0,73.0)))},
            FollowPath(paths[6]), //shoot first spike
            Delay(0.6),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[3],true,1.0), //intake gate
                InstantCommand { Rollers.run(0.3, 1.0) }
            ),
            Delay(0.7),
            InstantCommand { Shooter.setFlywheelManualVelocity(Shooter.getVelocity(Pose(55.0,73.0)))},
            ParallelGroup(
                FollowPath(paths[4],true,1.0), //shoot gate
                InstantCommand { Delay(1.0) },
                InstantCommand { Rollers.run(0.0,0.0)},
            ),
            Delay(0.6),
            Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[7]), //intake third spike
                InstantCommand { Rollers.run(0.3, 1.0) }
            ),
            InstantCommand { Rollers.run(0.0,0.0)},
            InstantCommand { Shooter.setFlywheelManualVelocity(Shooter.getVelocity(Pose(56.0,76.0)))},
            FollowPath(paths[8]), //shoot third spike
            Delay(0.6),
            Load . shootTripleCommand,
            FollowPath(paths[9]) //leave
        )
        main.schedule()
    }
    fun buildPaths() {
        paths = arrayOf()
        val startPose: Pose = Pose(34.0,132.9,Math.toRadians(-180.0))
        val shootPose: Pose = Pose(84.0,85.0).mirror()
        val middleIntakeEndPose: Pose = Pose(128.5,56.0,Math.toRadians(-20.0)).mirror()
        val middleIntakeControl: Pose = Pose(103.75, 65.5).mirror()

        val gateIntakePose: Pose = Pose(11.5, 58.910, Math.toRadians(147.731))

        val topIntakePose: Pose = Pose(120.0,85.0,0.0).mirror()

        val lastIntakePose: Pose = Pose(125.0,33.0,Math.toRadians(-15.0)).mirror()
        val lastIntakeControl: Pose = Pose(90.0,40.0).mirror()

        val lastShootPose: Pose = Pose(81.0,101.0,Math.toRadians(-90.0)).mirror()

        /*al path1 = follower.pathBuilder()
            .addPath(BezierLine(startPose, shootPose))
            .setLinearHeadingInterpolation(startPose.heading, Math.toRadians(-140.0))
            .build()
        val path2 = follower.pathBuilder()
            .addPath(BezierCurve(shootPose, middleIntakeControl, middleIntakeEndPose))
            .setLinearHeadingInterpolation(Math.toRadians(-140.0),middleIntakeEndPose.heading)
            .build()
        val path3 = follower.pathBuilder()
            .addPath(BezierLine(middleIntakeControl, shootPose))
            .setConstantHeadingInterpolation(Math.toRadians(-140.0))
            .build()
        val path4 = follower.pathBuilder()
            .addPath(BezierLine(shootPose, gateIntakePose))
            .setLinearHeadingInterpolation(Math.toRadians(-150.0),gateIntakePose.heading)
            .build()
        val path5 = follower.pathBuilder()
            .addPath(BezierLine(gateIntakePose, shootPose))
            .setLinearHeadingInterpolation(gateIntakePose.heading, Math.toRadians(-150.0))
            .build()
        val path6 = follower.pathBuilder()
            .addPath(BezierLine(shootPose, topIntakePose))
            .setConstantHeadingInterpolation(PI)
            .build()
        val path7 = follower.pathBuilder()
            .addPath(BezierLine(topIntakePose, shootPose))
            .setLinearHeadingInterpolation(PI,Math.toRadians(-135.0))
            .build()
        val path8 = follower.pathBuilder()
            .addPath(BezierCurve(shootPose, lastIntakeControl, lastIntakePose))
            .setLinearHeadingInterpolation(Math.toRadians(-90.0),Math.toRadians(-165.0))
            .build()
        val path9 = follower.pathBuilder()
            .addPath(BezierCurve(lastIntakePose, Pose(83.0,68.5).mirror(),lastShootPose))
            .setLinearHeadingInterpolation(Math.toRadians(-165.0),lastShootPose.heading)
            .build()*/
        val shootpreload = follower.pathBuilder().addPath(
            BezierLine(
                Pose(34.000, 132.900),

                Pose(55.000, 73.000)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(-180.0), Math.toRadians(-157.0))

            .build()

        val intakesecondspike = follower.pathBuilder().addPath(
            BezierLine(
                Pose(55.000, 73.000),

                Pose(15.400, 56.200)
            )
        ).setTangentHeadingInterpolation()

            .build()

        val shootsecondspike = follower.pathBuilder().addPath(
            BezierLine(
                Pose(15.400, 56.200),

                Pose(55.000, 73.000)
            )
        ).setTangentHeadingInterpolation()
            .setReversed()
            .build()

        val gateintake = follower.pathBuilder().addPath(
            BezierLine(
                Pose(55.000, 73.000),

                Pose(10.500, 58.000)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(-157.0), Math.toRadians(147.731))
            .addParametricCallback(0.8) {follower.setMaxPower(0.2)}
            .addParametricCallback(0.94) {follower.setMaxPower(1.0)}
            .build()

        val shootgate1 = follower.pathBuilder().addPath(
            BezierLine(
                Pose(10.500, 58.000),

                Pose(55.000, 73.000)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(147.731), Math.toRadians(160.0))
            .addParametricCallback(0.05) {follower.setMaxPower(1.0)}
            .build()

        val intakefirstspike = follower.pathBuilder().addPath(
            BezierLine(
                Pose(55.000, 73.000),

                Pose(19.000, 85.000)
            )
        ).setTangentHeadingInterpolation()

            .build()

        val shootfirstspike = follower.pathBuilder().addPath(
            BezierLine(
                Pose(19.000, 85.000),

                Pose(55.000, 73.000)
            )
        ).setTangentHeadingInterpolation()
            .setReversed()
            .build()

        val intakethirdspike = follower.pathBuilder().addPath(
            BezierCurve(
                Pose(55.000, 73.000),
                Pose(55.000, 32.000),
                Pose(45.000, 32.000),
                Pose(17.000, 34.000)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(162.0), Math.toRadians(180.0))

            .build()

        val shootthirdspike = follower.pathBuilder().addPath(
            BezierLine(
                Pose(17.000, 34.000),

                Pose(56.000, 76.000)
            )
        ).setTangentHeadingInterpolation()
            .setReversed()
            .build()

        val leave = follower.pathBuilder().addPath(
            BezierLine(
                Pose(56.000, 76.000),

                Pose(52.500, 72.500)
            )
        ).setTangentHeadingInterpolation()

            .build()
        paths += shootpreload
        paths += intakesecondspike
        paths += shootsecondspike
        paths += gateintake
        paths += shootgate1
        paths += intakefirstspike
        paths += shootfirstspike
        paths += intakethirdspike
        paths += shootthirdspike
        paths += leave

    }

    override fun onUpdate() {
        Shooter.update()
        telemetry.update()
    }
    override fun onStop() { ROBOT.currTeleOpStartPose = follower.pose }
}
