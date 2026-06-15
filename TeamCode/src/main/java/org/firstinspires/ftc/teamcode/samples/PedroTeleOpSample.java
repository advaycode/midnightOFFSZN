package org.firstinspires.ftc.teamcode.samples;


import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// CommandOpMode provides a command scheduler; used here even in TeleOp so subsystems can be registered later
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// Minimal TeleOp using Pedro Pathing's field-centric drive — good starting point for a competition TeleOp
@TeleOp
public class PedroTeleOpSample extends CommandOpMode {
    Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    @Override
    public void initialize() {
        // Build follower from tuned Constants and enable teleop drive mode
        follower = Constants.createFollower(hardwareMap);
        super.reset();

        follower.startTeleopDrive();
    }

    @Override
    public void run() {
        super.run();

        /* Robot-Centric Drive — last arg true means stick forward = robot forward
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        */

        // Field-Centric Drive — last arg false = field-centric, so stick forward always = field forward regardless of robot heading
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        // update() re-runs the localizer and PIDF controllers then writes motor powers each loop
        follower.update();

        // Stream current pose estimate to driver station telemetry
        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }
}
