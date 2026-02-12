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
        var pos1 = 1-0.96875
        @JvmField
        var pos2 = 0.5
    }
    private val left_tilt: ServoEx = ServoEx("lt", -0.1)
    private val right_t: ServoEx = ServoEx("ft",-0.1)


    override fun onUpdate() {
        left_tilt.position = pos1
        right_t.position = pos1
    }

}