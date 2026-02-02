package org.firstinspires.ftc.teamcode.TeleOp

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.hardware.impl.ServoEx
@Configurable
@TeleOp
class servo_tester: NextFTCOpMode() {
    init {

    }
    companion object {
        @JvmField
        var a = 0.0
    }
    private val tg: ServoEx = ServoEx("rm", -0.1)
    private val rg: ServoEx = ServoEx("lm", -0.1)


    override fun onUpdate() {
        tg.position = 0.0
        rg.position = 1.0
    }

}