package org.firstinspires.ftc.teamcode.WebTest

import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.Util.addSubsystems

class TestOpMode: NextFTCOpMode() {
    init {
        addSubsystems()
    }

    override fun onUpdate() {
        telemetry.addLine(YAMLWebHandler.getRawData("test.txt"))
    }
}