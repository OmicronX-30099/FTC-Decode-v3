package org.firstinspires.ftc.teamcode.Auto.Red

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.Constants.PedroConstants
import org.firstinspires.ftc.teamcode.Systems.Load.Load
import org.firstinspires.ftc.teamcode.Systems.Shooter.Shooter
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.includePedro

@Autonomous(name = "Sorted Red Auto", group = "Autos", preselectTeleOp = "Red TeleOp")
class SortedAuto: NextFTCOpMode() {
    init {
        addSubsystems(Shooter, Load)
        includePedro(PedroConstants::createFollower)
    }

    override fun onInit() {
        Shooter.reset()
        ROBOT.currStage = Stage.TELEOP
        ROBOT.currAlliance = Alliance.RED
    }
}