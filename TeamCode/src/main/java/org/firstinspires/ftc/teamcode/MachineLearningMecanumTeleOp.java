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
 * Mecanum TeleOp with active RPM equalization.
 *
 * Press LEFT BUMPER to toggle equalization on/off.
 * When ON: each motor's power is nudged every loop so all four stay at the
 * same RPM. Works best going straight — equalizationKp and rpmSmoothing are
 * tunable live in Panels under the "MachineLearningMecanumTeleOp" group.
 */
@Configurable
@TeleOp(name = "machineLearning_mecanum_teleop", group = "TeleOp")
public class MachineLearningMecanumTeleOp extends LinearOpMode {

    /** Power correction per RPM of error. Raise if equalization feels sluggish. */
    public static double equalizationKp = 0.0008;

    /** EMA smoothing for RPM readings (0 = raw, higher = smoother but laggier). */
    public static double rpmSmoothing = 0.35;

    private static final double TICKS_PER_REV    = 383.6;
    private static final double NORMAL_SPEED     = 1.0;
    private static final double SLOW_SPEED       = 0.35;
    private static final double BRAKE_GAIN       = 0.0003;
    private static final double STICK_DEADBAND   = 0.05;
    private static final double MIN_RPM_EQUALIZE = 15.0; // ignore noise below this

    @Override
    public void runOpMode() {
        // Retrieve all four drive motors by their hardware map names
        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx bl = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx br = hardwareMap.get(DcMotorEx.class, "rightRear");

        // Reset encoders, run open-loop, and brake on zero power
        for (DcMotorEx m : new DcMotorEx[]{fl, bl, fr, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Left side reversed so all wheels drive in the same direction for positive power
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        // RPM tracking state — snapshot encoder counts and timestamp each loop to compute RPM
        int[] prevPos = {
            fl.getCurrentPosition(), bl.getCurrentPosition(),
            fr.getCurrentPosition(), br.getCurrentPosition()
        };
        long     prevMs = System.currentTimeMillis();
        double[] rpm    = new double[4]; // smoothed RPM per motor

        boolean equalizationOn = false;
        boolean lastLBumper    = false;

        // Pull the latest Panels values into static fields before waiting for start
        PanelsConfigurables.INSTANCE.refreshClass(this);
        telemetry.addData("Status", "Ready — Left Bumper toggles RPM equalization");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // ── Toggle equalization ───────────────────────────────────────────
            // Rising-edge detection on left bumper flips the equalization flag
            boolean lbNow = gamepad1.left_bumper;
            if (lbNow && !lastLBumper) equalizationOn = !equalizationOn;
            lastLBumper = lbNow;

            // ── Drive inputs ──────────────────────────────────────────────────
            double drive  =  gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotate = -gamepad1.right_stick_x;
            double speed  =  gamepad1.left_trigger > 0.1 ? SLOW_SPEED : NORMAL_SPEED;

            // Dead-band check — if all sticks are idle, apply active braking instead of driving
            boolean driverActive = Math.abs(drive)  > STICK_DEADBAND
                                || Math.abs(strafe) > STICK_DEADBAND
                                || Math.abs(rotate) > STICK_DEADBAND;

            double flPow, blPow, frPow, brPow;
            if (driverActive) {
                // Standard mecanum mixing: each wheel gets a combination of drive, strafe, and rotate
                flPow = (drive + strafe + rotate) * speed;
                blPow = (drive - strafe + rotate) * speed;
                frPow = (drive - strafe - rotate) * speed;
                brPow = (drive + strafe - rotate) * speed;
            } else {
                // When sticks are idle, oppose current motor velocity slightly to slow coast
                flPow = -fl.getVelocity() * BRAKE_GAIN;
                blPow = -bl.getVelocity() * BRAKE_GAIN;
                frPow = -fr.getVelocity() * BRAKE_GAIN;
                brPow = -br.getVelocity() * BRAKE_GAIN;
            }

            // Normalize so the largest wheel power is at most 1.0, preserving direction ratios
            double maxPow = Math.max(1.0,
                    Math.max(Math.abs(flPow),
                    Math.max(Math.abs(blPow),
                    Math.max(Math.abs(frPow), Math.abs(brPow)))));
            flPow /= maxPow;
            blPow /= maxPow;
            frPow /= maxPow;
            brPow /= maxPow;

            // ── Update smoothed RPM (EMA on position deltas) ──────────────────
            // EMA (exponential moving average) reduces encoder noise before equalization uses the value
            long now = System.currentTimeMillis();
            long dt  = now - prevMs;
            if (dt > 0) {
                int[] cur = {
                    fl.getCurrentPosition(), bl.getCurrentPosition(),
                    fr.getCurrentPosition(), br.getCurrentPosition()
                };
                double dtMin = dt / 60000.0; // ms → minutes for RPM formula
                double a     = rpmSmoothing;
                for (int i = 0; i < 4; i++) {
                    double raw = (cur[i] - prevPos[i]) / TICKS_PER_REV / dtMin;
                    rpm[i]     = a * rpm[i] + (1.0 - a) * raw;
                    prevPos[i] = cur[i];
                }
                prevMs = now;
            }

            // ── Active RPM equalization ───────────────────────────────────────
            // Nudge each motor toward the fleet average RPM using a proportional correction
            if (equalizationOn && driverActive) {
                double mean = (rpm[0] + rpm[1] + rpm[2] + rpm[3]) / 4.0;
                if (Math.abs(mean) >= MIN_RPM_EQUALIZE) {
                    flPow = clamp(flPow + (mean - rpm[0]) * equalizationKp);
                    blPow = clamp(blPow + (mean - rpm[1]) * equalizationKp);
                    frPow = clamp(frPow + (mean - rpm[2]) * equalizationKp);
                    brPow = clamp(brPow + (mean - rpm[3]) * equalizationKp);
                }
            }

            fl.setPower(flPow);
            bl.setPower(blPow);
            fr.setPower(frPow);
            br.setPower(brPow);

            // ── Telemetry ─────────────────────────────────────────────────────
            double mean = (rpm[0] + rpm[1] + rpm[2] + rpm[3]) / 4.0;
            double maxDev = 0;
            for (double r : rpm) maxDev = Math.max(maxDev, Math.abs(r - mean));

            telemetry.addData("Equalization [LB]", equalizationOn ? "ON" : "OFF");
            telemetry.addData("Speed mode", gamepad1.left_trigger > 0.1 ? "SLOW" : "NORMAL");
            telemetry.addLine();
            telemetry.addData("Mean RPM", String.format("%.0f", mean));
            telemetry.addData("Max deviation", String.format("%.0f RPM", maxDev));
            telemetry.addLine();
            telemetry.addData("FL", String.format("%.0f RPM", rpm[0]));
            telemetry.addData("BL", String.format("%.0f RPM", rpm[1]));
            telemetry.addData("FR", String.format("%.0f RPM", rpm[2]));
            telemetry.addData("BR", String.format("%.0f RPM", rpm[3]));
            telemetry.update();
        }
    }

    // Clamp helper — ensures motor power stays within the legal -1.0 to 1.0 range
    private static double clamp(double v) {
        return Math.max(-1.0, Math.min(1.0, v));
    }
}
