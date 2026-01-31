package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.hardware.impl.ServoEx
@TeleOp
class servo_tester: NextFTCOpMode() {
    init {

    }
    private val leftservo: ServoEx = ServoEx("l_tilt",-0.1)
    private val rightservo: ServoEx = ServoEx("r_tilt",-0.1)
    override fun onStartButtonPressed() {
        leftservo.position = 0.5
        rightservo.position = 0.5
    }
}