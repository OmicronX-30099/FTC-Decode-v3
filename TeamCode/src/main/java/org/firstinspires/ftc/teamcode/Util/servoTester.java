package org.firstinspires.ftc.teamcode.Util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "ServoTest")
public class servoTester extends OpMode {
    
    public static String c_name = "lt";
    public static String b_name = "ft";
    public static double pos = 0.0;
    public static double pos1 = 0.0;
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        Servo test = hardwareMap.get(Servo.class, c_name);
        Servo test1 = hardwareMap.get(Servo.class, b_name);
        test.setPosition(pos);
        test1.setPosition(pos1);
    }
}
