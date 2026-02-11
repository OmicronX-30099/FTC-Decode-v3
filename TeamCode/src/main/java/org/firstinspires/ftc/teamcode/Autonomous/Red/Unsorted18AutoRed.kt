package org.firstinspires.ftc.teamcode.Autonomoous.Red

@Autonomous(name = "Unsorted 18-ball Red", group = "Unsorted Auto", preselectTeleop = "Red TeleOp")
class Unsorted18AutoRed(): NextFTCOpMode() {
    init {
        addSubsystems(Load, Shooter, Miscellaneous)
        includePedro(PedroConstants::createFollower)
    }

    private var paths = Array<PathChain> = arrayOf()
    
    override fun onInit() {
        ROBOT.currAlliance = Alliance.RED
        ROBOT.currStage = Stage.AUTONOMOUS
        ROBOT.currStage.useFlywheelVel = true
    }
    override fun onStartButtonPressed() {
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
