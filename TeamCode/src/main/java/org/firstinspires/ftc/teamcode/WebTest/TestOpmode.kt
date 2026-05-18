package org.firstinspires.ftc.teamcode.WebTest

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.Util.addSubsystems

@TeleOp()
class TestOpMode: NextFTCOpMode() {
    init {
        addSubsystems()
    }

    override fun onUpdate() {
        telemetry.addLine(YAMLWebHandler.getRawData("test.txt"))
        telemetry.update()
    }
}