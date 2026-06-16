package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends OpMode {

    private Servo testServo;
    private double currentPosition = 0.5;
    boolean lastPressed = false;
    boolean lastLeftPressed = false;

    @Override
    public void init() {
        testServo = hardwareMap.get(Servo.class, "testServo");
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_left && !lastLeftPressed) {
        currentPosition = currentPosition - 0.01;
           currentPosition = Math.max(0.0, Math.min(1.0, currentPosition));
            testServo.setPosition(currentPosition);
        }
        if (gamepad1.dpad_up) {
            currentPosition = 0.5;
            testServo.setPosition(currentPosition);
        }
        if(gamepad1.dpad_right && !lastPressed) {
            currentPosition = currentPosition + 0.01;
            currentPosition = Math.max(0.0, Math.min(1.0, currentPosition));
            testServo.setPosition(currentPosition);
        }

        telemetry.addData("Servo Position", currentPosition * 130);
        telemetry.update();
        lastPressed = gamepad1.dpad_right;
        lastLeftPressed = gamepad1.dpad_left;
    }


}
