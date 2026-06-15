package org.firstinspires.ftc.teamcode;

// PanelsConfigurables lets you edit public static fields live from the Panels dashboard
import com.bylazar.configurables.PanelsConfigurables;
// @Configurable exposes all public static fields in this class to the Panels dashboard
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
// DcMotorEx extends DcMotor with encoder velocity access needed for RPM calculation
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
        // Retrieve all four drive motors by their hardware map names
        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx bl = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx br = hardwareMap.get(DcMotorEx.class, "rightRear");

        // Reset encoders to zero, then run open-loop (no PID velocity control) and brake on zero power
        for (DcMotorEx m : new DcMotorEx[]{fl, bl, fr, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Left side reversed so positive power drives all wheels in the same physical direction
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        // Snapshot encoder counts and timestamp to compute RPM delta each loop
        int[] prevPos = {
            fl.getCurrentPosition(), bl.getCurrentPosition(),
            fr.getCurrentPosition(), br.getCurrentPosition()
        };
        long prevMs = System.currentTimeMillis();
        double[] rpm = new double[4];

        // Pull the latest Panels values into static fields before waiting for start
        PanelsConfigurables.INSTANCE.refreshClass(this);
        telemetry.addData("Status", "Ready — set power targets in Panels then press Start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Apply each motor's power target (converted from percent to 0–1 range)
            fl.setPower(flPower / 100.0);
            bl.setPower(blPower / 100.0);
            fr.setPower(frPower / 100.0);
            br.setPower(brPower / 100.0);

            // Compute RPM from encoder delta over elapsed time: (ticks / ticksPerRev) / minutes
            long now = System.currentTimeMillis();
            long dt  = now - prevMs;
            if (dt > 0) {
                int[] cur = {
                    fl.getCurrentPosition(), bl.getCurrentPosition(),
                    fr.getCurrentPosition(), br.getCurrentPosition()
                };
                double dtMin = dt / 60000.0; // ms → minutes
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

        // Zero all motors on exit
        for (DcMotorEx m : new DcMotorEx[]{fl, bl, fr, br}) m.setPower(0);
    }
}
