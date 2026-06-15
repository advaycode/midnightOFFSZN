package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends OpMode {

    private Servo testServo;

    @Override
    public void init() {
        testServo = hardwareMap.get(Servo.class, "testServo");
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_left) {
        testServo.setPosition(0.0);
        }
        if (gamepad1.dpad_right) {
            testServo.setPosition(0.7);
        }
    }


}
