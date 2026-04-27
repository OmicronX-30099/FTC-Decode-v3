package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "BeamBreak Test", group = "Util")
public class BeamBreakTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DigitalChannel bb1 = hardwareMap.get(DigitalChannel.class, "bb1");
        DigitalChannel bb2 = hardwareMap.get(DigitalChannel.class, "bb2");
        DigitalChannel bb3 = hardwareMap.get(DigitalChannel.class, "bb3");
        DigitalChannel bb4 = hardwareMap.get(DigitalChannel.class, "bb4");
        DigitalChannel bb5 = hardwareMap.get(DigitalChannel.class, "bb5");
        DigitalChannel bb6 = hardwareMap.get(DigitalChannel.class, "bb6");

        DcMotor intake = hardwareMap.get(DcMotor.class, "t");
        DcMotor transfer = hardwareMap.get(DcMotor.class, "i");
        Servo gate = hardwareMap.get(Servo.class, "shooter_gate");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all to input mode
        bb1.setMode(DigitalChannel.Mode.INPUT);
        bb2.setMode(DigitalChannel.Mode.INPUT);
        bb3.setMode(DigitalChannel.Mode.INPUT);
        bb4.setMode(DigitalChannel.Mode.INPUT);
        bb5.setMode(DigitalChannel.Mode.INPUT);
        bb6.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Status", "Initialized. Press play to start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0) {
                intake.setPower(1);
                transfer.setPower(1);
            } else if (gamepad1.left_trigger > 0) {
                intake.setPower(-1);
                transfer.setPower(-1);
            } else {
                intake.setPower(0);
                transfer.setPower(0);
            }

            if (gamepad1.a) {
                gate.setPosition(0.2); // unlock
            } else if (gamepad1.b) {
                gate.setPosition(0.5); // lock
            }

            telemetry.addData("Intake Power", intake.getPower());
            telemetry.addData("Transfer Power", transfer.getPower());
            telemetry.addData("Gate Position", gate.getPosition());

            // In FTC, .getState() returns true if the beam is NOT broken (voltage high)
            // and false if the beam IS broken (voltage low/grounded).
            
            telemetry.addData("BB1", bb1.getState() ? "CLEAN" : "BROKEN");
            telemetry.addData("BB2", bb2.getState() ? "CLEAN" : "BROKEN");
            telemetry.addData("BB3", bb3.getState() ? "CLEAN" : "BROKEN");
            telemetry.addData("BB4", bb4.getState() ? "CLEAN" : "BROKEN");
            telemetry.addData("BB5", bb5.getState() ? "CLEAN" : "BROKEN");
            telemetry.addData("BB6", bb6.getState() ? "CLEAN" : "BROKEN");
            
            telemetry.addLine("\n--- Grouped States ---");
            telemetry.addData("Pos 1 (bb1 || bb2)", (!bb1.getState() || !bb2.getState()) ? "OCCUPIED" : "EMPTY");
            telemetry.addData("Pos 2 (bb3 || bb4)", (!bb3.getState() || !bb4.getState()) ? "OCCUPIED" : "EMPTY");
            telemetry.addData("Pos 3 (bb5 || bb6)", (!bb5.getState() || !bb6.getState()) ? "OCCUPIED" : "EMPTY");

            telemetry.update();
        }
    }
}
