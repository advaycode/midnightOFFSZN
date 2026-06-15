package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
// PanelsTelemetry streams debug values to the Panels dashboard instead of the driver station
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// Main competition TeleOp — robot-centric mecanum drive via Pedro Pathing follower with optional slow mode
@Configurable
@TeleOp(name = "Mecanum TeleOp", group = "TeleOp")
public class MecanumTeleOp extends OpMode {

    private Follower follower;
    // TelemetryManager wraps Panels telemetry so debug values appear on the phone dashboard
    private TelemetryManager telemetryM;

    // Fraction of full speed applied when left trigger is held — adjustable live in Panels
    public static double slowMultiplier = 0.35;

    @Override
    public void init() {
        // Create follower from tuned Constants and reset pose to field origin
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        // Switch follower from autonomous path mode to manual teleop drive mode
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // update() runs the localizer and applies any heading correction each loop
        follower.update();

        // Left trigger held → slow mode multiplier; otherwise full speed
        double mult = gamepad1.left_trigger > 0.1 ? slowMultiplier : 1.0;

        // setTeleOpDrive: (forward, strafe, rotate, robotCentric) — true = stick forward means robot forward
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y  * mult,
                -gamepad1.left_stick_x  * mult,
                -gamepad1.right_stick_x * mult,
                true
        );

        // Stream pose and slow-mode state to the Panels dashboard each loop
        telemetryM.debug("x",       follower.getPose().getX());
        telemetryM.debug("y",       follower.getPose().getY());
        telemetryM.debug("heading", follower.getPose().getHeading());
        telemetryM.debug("slow",    gamepad1.left_trigger > 0.1);
        telemetryM.update();
    }
}
