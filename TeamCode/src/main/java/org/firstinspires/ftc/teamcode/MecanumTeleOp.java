package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "Mecanum TeleOp", group = "TeleOp")
public class MecanumTeleOp extends OpMode {

    private Follower follower;
    private TelemetryManager telemetryM;

    public static double slowMultiplier = 0.35;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        double mult = gamepad1.left_trigger > 0.1 ? slowMultiplier : 1.0;

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y  * mult,
                -gamepad1.left_stick_x  * mult,
                -gamepad1.right_stick_x * mult,
                true
        );

        telemetryM.debug("x",       follower.getPose().getX());
        telemetryM.debug("y",       follower.getPose().getY());
        telemetryM.debug("heading", follower.getPose().getHeading());
        telemetryM.debug("slow",    gamepad1.left_trigger > 0.1);
        telemetryM.update();
    }
}
