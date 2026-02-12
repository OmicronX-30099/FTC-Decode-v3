package org.firstinspires.ftc.teamcode.Autonomoous.Red

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
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.includePedro

@Autonomous(name = "Unsorted 18-ball Red", group = "Unsorted Auto", preselectTeleOp = "Red TeleOp")
class Unsorted18AutoRed(): NextFTCOpMode() {
    init {
        addSubsystems(Load, Shooter, Miscellaneous)
        includePedro(PedroConstants::createFollower)
    }

    private var paths: Array<PathChain> = arrayOf()
    
    override fun onInit() {
        ROBOT.currAlliance = Alliance.RED
        ROBOT.currStage = Stage.AUTONOMOUS
        ROBOT.currStage.useFlywheelVel = true
    }
    override fun onStartButtonPressed() {
        follower.setStartingPose(Pose(0.0, 0.0, 0.0))
        buildPaths()
        val main = SequentialGroup(
            FollowPath(paths[0]),
            Delay(0.1),
                    Load.shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[1]),
                InstantCommand { Rollers.run(0.3, 1.0) }
            ),
            Delay(0.1),
            FollowPath(paths[2]),
            Delay(0.2),
                    Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[3]),
                InstantCommand { Rollers.run(0.3, 1.0) }
            ),
            Delay(0.1),
            FollowPath(paths[4]),
            Delay(0.2),
                    Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[5]),
                InstantCommand { Rollers.run(0.3, 1.0) }
            ),
            Delay(0.1),
            FollowPath(paths[6]),
            Delay(0.2),
                    Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[7]),
                InstantCommand { Rollers.run(0.3, 1.0) }
            ),
            Delay(0.1),
            FollowPath(paths[8]),
            Delay(0.2),
                    Load . shootTripleCommand,
            ParallelGroup(
                FollowPath(paths[9]),
                InstantCommand { Rollers.run(0.3, 1.0) }
            ),
            Delay(0.1),
            FollowPath(paths[10]),
            Delay(0.2),
                    Load . shootTripleCommand
        )
    }
    fun buildPaths() {
        paths = arrayOf()
        
    }
}
