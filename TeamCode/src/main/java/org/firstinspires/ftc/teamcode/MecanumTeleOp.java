package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Mecanum TeleOp", group = "TeleOp")
public class MecanumTeleOp extends LinearOpMode {

    // ── Motor config names — match these in the Driver Hub hardware config ────
    private static final String MOTOR_FL = "leftFront";
    private static final String MOTOR_BL = "leftRear";
    private static final String MOTOR_FR = "rightFront";
    private static final String MOTOR_BR = "rightRear";

    // ── Speed multipliers ─────────────────────────────────────────────────────
    private static final double NORMAL_SPEED  = 1.0;
    private static final double SLOW_SPEED    = 0.35;  // hold left trigger to activate

    @Override
    public void runOpMode() {
        DcMotor fl = hardwareMap.get(DcMotor.class, MOTOR_FL);
        DcMotor bl = hardwareMap.get(DcMotor.class, MOTOR_BL);
        DcMotor fr = hardwareMap.get(DcMotor.class, MOTOR_FR);
        DcMotor br = hardwareMap.get(DcMotor.class, MOTOR_BR);

        // ── Run mode — must be set explicitly or setPower() may be ignored ──────
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ── Motor directions (flip if a side drives backwards) ────────────────
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
            // PS5: left stick Y is negative when pushed forward
            double drive  = gamepad1.left_stick_y;   // forward / back
            double strafe =  gamepad1.left_stick_x;   // left / right
            double rotate =  gamepad1.right_stick_x;  // turn

            // left trigger = slow mode (PS5 left trigger is left_trigger float 0..1)
            double speed = gamepad1.left_trigger > 0.1 ? SLOW_SPEED : NORMAL_SPEED;

            double flPow = (drive + strafe + rotate) * speed;
            double blPow = (drive - strafe + rotate) * speed;
            double frPow = (drive - strafe - rotate) * speed;
            double brPow = (drive + strafe - rotate) * speed;

            // Normalize so no value exceeds 1.0 while preserving ratio
            double max = Math.max(1.0,
                    Math.max(Math.abs(flPow),
                    Math.max(Math.abs(blPow),
                    Math.max(Math.abs(frPow), Math.abs(brPow)))));

            fl.setPower(flPow / max);
            bl.setPower(blPow / max);
            fr.setPower(frPow / max);
            br.setPower(brPow / max);

            telemetry.addData("Gamepad connected", gamepad1.getGamepadId() != -1);
            telemetry.addData("Raw sticks LY/LX/RX", "%.2f / %.2f / %.2f",
                    gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            telemetry.addData("Speed mode", gamepad1.left_trigger > 0.1 ? "SLOW" : "NORMAL");
            telemetry.addData("Drive / Strafe / Rotate", "%.2f / %.2f / %.2f", drive, strafe, rotate);
            telemetry.addData("FL / BL / FR / BR", "%.2f / %.2f / %.2f / %.2f",
                    flPow / max, blPow / max, frPow / max, brPow / max);
            telemetry.update();
        }
    }
}
