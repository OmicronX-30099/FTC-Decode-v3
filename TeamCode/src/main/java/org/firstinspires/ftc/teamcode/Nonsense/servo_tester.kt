package org.firstinspires.ftc.teamcode.Nonsense

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
        var pos1 = 0.4
        @JvmField
        var pos2 = 0.5
    }
    private val left_tilt: ServoEx = ServoEx("sg", -0.1)


    override fun onUpdate() {
        left_tilt.position = pos1
    }

}