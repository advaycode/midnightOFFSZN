package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Mecanum TeleOp", group = "TeleOp")
public class MecanumTeleOp extends LinearOpMode {

    private static final String MOTOR_FL = "leftFront";
    private static final String MOTOR_BL = "leftRear";
    private static final String MOTOR_FR = "rightFront";
    private static final String MOTOR_BR = "rightRear";

    private static final double NORMAL_SPEED   = 1.0;
    private static final double SLOW_SPEED     = 0.35;

    // Predictive braking — scales counter-force to velocity when sticks released
    private static final double BRAKE_GAIN     = 0.0003;  // tune up if braking feels weak
    private static final double STICK_DEADBAND = 0.05;

    @Override
    public void runOpMode() {
        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, MOTOR_FL);
        DcMotorEx bl = hardwareMap.get(DcMotorEx.class, MOTOR_BL);
        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, MOTOR_FR);
        DcMotorEx br = hardwareMap.get(DcMotorEx.class, MOTOR_BR);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double drive  =  gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotate = -gamepad1.right_stick_x;
            double speed  =  gamepad1.left_trigger > 0.1 ? SLOW_SPEED : NORMAL_SPEED;

            boolean driverActive = Math.abs(drive)  > STICK_DEADBAND
                                || Math.abs(strafe) > STICK_DEADBAND
                                || Math.abs(rotate) > STICK_DEADBAND;

            double flPow, blPow, frPow, brPow;

            if (driverActive) {
                flPow = (drive + strafe + rotate) * speed;
                blPow = (drive - strafe + rotate) * speed;
                frPow = (drive - strafe - rotate) * speed;
                brPow = (drive + strafe - rotate) * speed;
            } else {
                // Predictive braking: counter-force proportional to each motor's velocity
                flPow = -fl.getVelocity() * BRAKE_GAIN;
                blPow = -bl.getVelocity() * BRAKE_GAIN;
                frPow = -fr.getVelocity() * BRAKE_GAIN;
                brPow = -br.getVelocity() * BRAKE_GAIN;
            }

            double max = Math.max(1.0,
                    Math.max(Math.abs(flPow),
                    Math.max(Math.abs(blPow),
                    Math.max(Math.abs(frPow), Math.abs(brPow)))));

            fl.setPower(flPow / max);
            bl.setPower(blPow / max);
            fr.setPower(frPow / max);
            br.setPower(brPow / max);

            telemetry.addData("Gamepad connected", gamepad1.getGamepadId() != -1);
            telemetry.addData("Speed mode", gamepad1.left_trigger > 0.1 ? "SLOW" : "NORMAL");
            telemetry.addData("Braking", !driverActive);
            telemetry.update();
        }
    }
}
