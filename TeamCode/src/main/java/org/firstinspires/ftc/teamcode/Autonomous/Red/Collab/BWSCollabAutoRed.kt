package org.firstinspires.ftc.teamcode.Autonomous.Red.Collab

@Autonomous(name = "Red BWS Collab Auto", group = "BWS Collab Auto", preselectTeleop = "Red TeleOp")
class BWSCollabAutoRed(): NextFTCOpMode() {
    init {
        addSubsystems(Shooter, Load, Miscellaneous)
        includePedro(PedroConstants::createFollower)
    }
    private var paths: Array<PathChain> = arrayOf()

    override fun onInit() {
        ROBOT.currAlliance = Alliance.RED
        ROBOT.currStage = Stage.TELEOP
        ROBOT.currStage.useFlywheelVel = false
    }
    override fun onStartButtonPressed() {
        buildPaths()
        follower.setStartingPose(Pose(86.750,7.5,Math.toRadians(90.0)))

        val main = SequentialGroup(
            Delay(1.3),
            Load.shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[0], holdEnd = true, 1.0),
                SequentialGroup (
                    InstantCommand { Rollers.run(0.3,1.0) }
                )
            ),
            Delay(0.5),
            InstantCommand { Rollers.stop() },
            FollowPath(paths[1]),
            Delay(0.5),
            Load.shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[2]),//intake corner p1
                SequentialGroup(
                    InstantCommand { Rollers.run(0.3,1.0) }
                )
            ),
            FollowPath(paths[3]),//intake corner p2
            FollowPath(paths[4]),//intake corner p3
            InstantCommand { Rollers.stop() },
            FollowPath(paths[5]),//shootcorner
            Delay(0.5),
            Load.shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[6]),//intake tunnel
                SequentialGroup(
                    InstantCommand { Rollers.run(0.3,1.0) }
                )
            ),
            Delay(0.15),
            InstantCommand { Rollers.stop() },
            FollowPath(paths[7]),//shoot tunnel
            Delay(0.5),
            Load.shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[2]),//intake corner p1
                SequentialGroup(
                    InstantCommand { Rollers.run(0.3,1.0) }
                )
            ),
            FollowPath(paths[3]),//intake corner p2
            FollowPath(paths[4]),//intake corner p3
            InstantCommand { Rollers.stop() },
            FollowPath(paths[5]),//shoot corner
            Delay(0.5),
            Load.shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[6]),//intake tunnel
                SequentialGroup(
                    InstantCommand { Rollers.run(0.3,1.0) }
                )
            ),
            Delay(0.15),
            InstantCommand { Rollers.stop() },
            FollowPath(paths[7]),//shoot tunnel
            Delay(0.5),
            Load.shootTripleCommand,
            FollowPath(paths[8],true,0.5)//leave
        )
        main.schedule()
    }

    override fun onUpdate() {
        Shooter.updateShooter()
        telemetry.update()
    }
    fun buildPaths() {
        paths = arrayof()
        val intakespike3 = follower.pathBuilder().addPath(
            BezierCurve(
                Pose(86.750, 7.500),
                Pose(92.000, 34.000),
                Pose(130.000, 35.000)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(90.0), Math.toRadians(0.0))
            .build()
        val shootspike3 = follower.pathBuilder().addPath(
            BezierLine(
                Pose(130.000, 35.000),
                Pose(90.000, 15.000)
            )
        ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(0.0))
            .build()
        val intakecornerp1 = follower.pathBuilder().addPath(
            BezierLine(
                Pose(90.000, 15.000),
                Pose(127.000, 11.000)
            )
        ).setTangentHeadingInterpolation()
            .build()
        val intakecornerp2 = follower.pathBuilder().addPath(
            BezierLine(
                Pose(127.000, 11.000),
                Pose(124.000, 11.500)
            )
        ).setTangentHeadingInterpolation()
            .setReversed()
            .build()
        val intakecornerp3 = follower.pathBuilder().addPath(
            BezierLine(
                Pose(124.000, 11.500),
                Pose(127.000, 11.000)
            )
        ).setTangentHeadingInterpolation()
            .build()
        val shootcorner = follower.pathBuilder().addPath(
            BezierLine(
                Pose(127.000, 11.000),
                Pose(90.000, 15.000)
            )
        ).setTangentHeadingInterpolation()
            .setReversed()
            .build()
        val intaketunnel = follower.pathBuilder().addPath(
            BezierLine(
                Pose(90.000, 15.000),
                Pose(125.000, 25.000)
            )
        ).setTangentHeadingInterpolation()
            .build()
        val shoottunnel = follower.pathBuilder().addPath(
            BezierLine(
                Pose(125.000, 25.000),
                Pose(90.000, 15.000)
            )
        ).setTangentHeadingInterpolation()
            .setReversed()
            .build()
        val leave = follower.pathBuilder().addPath(
            BezierLine(
                Pose(90.000, 15.000),
                Pose(100.000, 15.000)
            )
        ).setTangentHeadingInterpolation()
            .build()
        paths += intakespike3
        paths += shootspike3
        paths += intakecornerp1
        paths += intakecornerp2
        paths += intakecornerp3
        paths += shootcorner
        paths += intaketunnel
        paths += shoottunnel
        paths += leave
    }
}
