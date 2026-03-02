package org.firstinspires.ftc.teamcode.Util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "ServoTest")
public class servoTester extends OpMode {
    
    public static String c_name = "";
    public static double pos = 0.0;
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        Servo test = hardwareMap.get(Servo.class, c_name);
        test.setPosition(pos);
    }
}
