package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Set power % per motor in Panels → reads back actual RPM from encoders on driver station.
 */
@Configurable
@TeleOp(name = "Motor Test", group = "TeleOp")
public class MotorTest extends LinearOpMode {

    // Power targets (0 to 100 %) — set these in Panels
    public static double flPower = 0;
    public static double blPower = 0;
    public static double frPower = 0;
    public static double brPower = 0;

    private static final double TICKS_PER_REV = 383.6; // GoBilda 435 RPM

    @Override
    public void runOpMode() {
        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx bl = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx br = hardwareMap.get(DcMotorEx.class, "rightRear");

        for (DcMotorEx m : new DcMotorEx[]{fl, bl, fr, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        int[] prevPos = {
            fl.getCurrentPosition(), bl.getCurrentPosition(),
            fr.getCurrentPosition(), br.getCurrentPosition()
        };
        long prevMs = System.currentTimeMillis();
        double[] rpm = new double[4];

        PanelsConfigurables.INSTANCE.refreshClass(this);
        telemetry.addData("Status", "Ready — set power targets in Panels then press Start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            fl.setPower(flPower / 100.0);
            bl.setPower(blPower / 100.0);
            fr.setPower(frPower / 100.0);
            br.setPower(brPower / 100.0);

            long now = System.currentTimeMillis();
            long dt  = now - prevMs;
            if (dt > 0) {
                int[] cur = {
                    fl.getCurrentPosition(), bl.getCurrentPosition(),
                    fr.getCurrentPosition(), br.getCurrentPosition()
                };
                double dtMin = dt / 60000.0;
                for (int i = 0; i < 4; i++) {
                    rpm[i]     = (cur[i] - prevPos[i]) / TICKS_PER_REV / dtMin;
                    prevPos[i] = cur[i];
                }
                prevMs = now;
            }

            telemetry.addData("FL", String.format("%.0f%% → %.0f RPM", flPower, rpm[0]));
            telemetry.addData("BL", String.format("%.0f%% → %.0f RPM", blPower, rpm[1]));
            telemetry.addData("FR", String.format("%.0f%% → %.0f RPM", frPower, rpm[2]));
            telemetry.addData("BR", String.format("%.0f%% → %.0f RPM", brPower, rpm[3]));
            telemetry.update();
        }

        for (DcMotorEx m : new DcMotorEx[]{fl, bl, fr, br}) m.setPower(0);
    }
}
