package org.firstinspires.ftc.teamcode.Autonomoous.Red

@Autonomous(name = "Unsorted 18-ball Blue", group = "Unsorted Auto", preselectTeleop = "Blue TeleOp")
class Unsorted18AutoRed(): NextFTCOpMode() {
    init {
        addSubsystems(Load, Shooter, Miscellaneous)
        includePedro(PedroConstants::createFollower)
    }

    private var paths = Array<PathChain> = arrayOf()
    
    override fun onInit() {
        ROBOT.currAlliance = Alliance.BLUE
        ROBOT.currStage = Stage.AUTONOMOUS
        ROBOT.currStage.useFlywheelVel = true
    }
    override fun onStartButtonPressed() {
        follower.setStartingPose(Pose(0.0,0.0,0.0).mirror())
        buildPaths()
        val main = SequentialGroup(
            FollowPath(paths[0]),
            Delay(0.1)
            Load.tripleShootCommand,
            ParallelGroup(
                FollowPath(paths[1]),
                InstantCommand { Rollers.run(0.3,1.0) }
            ),
            Delay(0.1),
            FollowPath(paths[2]),
            Delay(0.2)
            Load.tripleShootCommand,
            ParallelGroup(
                FollowPath(paths[3]),
                InstantCommand { Rollers.run(0.3,1.0) }
            ),
            Delay(0.1),
            FollowPath(paths[4]),
            Delay(0.2)
            Load.tripleShootCommand,
            ParallelGroup(
                FollowPath(paths[5]),
                InstantCommand { Rollers.run(0.3,1.0) }
            ),
            Delay(0.1),
            FollowPath(paths[6]),
            Delay(0.2)
            Load.tripleShootCommand,
            ParallelGroup(
                FollowPath(paths[7]),
                InstantCommand { Rollers.run(0.3,1.0) }
            ),
            Delay(0.1),
            FollowPath(paths[8]),
            Delay(0.2)
            Load.tripleShootCommand,
            ParallelGroup(
                FollowPath(paths[9]),
                InstantCommand { Rollers.run(0.3,1.0) }
            ),
            Delay(0.1),
            FollowPath(paths[10]),
            Delay(0.2)
            Load.tripleShootCommand
        )
    }
    fun buildPaths() {
        paths = arrayOf()
        
    }
}
