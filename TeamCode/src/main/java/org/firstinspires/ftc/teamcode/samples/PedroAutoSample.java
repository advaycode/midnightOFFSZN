package org.firstinspires.ftc.teamcode.samples;

// Pedro Pathing path following and geometry types
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
// Marks this as an autonomous OpMode visible on the driver station
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// CommandOpMode from SolversLib provides a sequential command scheduler
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
// FollowPathCommand tells the follower to drive along a PathChain and finishes when the path is complete
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class PedroAutoSample extends CommandOpMode {
    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    // Named field positions — Pose holds X, Y (inches from field origin) and heading (radians)
    private final Pose startPose = new Pose(9, 111, Math.toRadians(-90));
    private final Pose scorePose = new Pose(16, 128, Math.toRadians(-45));
    private final Pose pickup1Pose = new Pose(30, 121, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(30, 131, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(45, 128, Math.toRadians(90));
    private final Pose parkPose = new Pose(68, 96, Math.toRadians(-90));

    // Path chains
    private PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3;
    private PathChain scorePickup1, scorePickup2, scorePickup3, park;

    // Builds all PathChains by linking Pose waypoints — BezierLine = straight segment, BezierCurve = smooth arc
    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        // BezierCurve with a control point smooths the corner between score and park
        park = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePose,
                        new Pose(68, 110), // Control point
                        parkPose)
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    // Mechanism commands - replace these with your actual subsystem commands
    // InstantCommand wraps a zero-duration lambda so it can be dropped into the command scheduler
    private InstantCommand openOuttakeClaw() {
        return new InstantCommand(() -> {
            // Example: outtakeSubsystem.openClaw();
        });
    }

    private InstantCommand grabSample() {
        return new InstantCommand(() -> {
            // Example: intakeSubsystem.grabSample();
        });
    }

    private InstantCommand scoreSample() {
        return new InstantCommand(() -> {
            // Example: outtakeSubsystem.scoreSample();
        });
    }

    private InstantCommand level1Ascent() {
        return new InstantCommand(() -> {
            // Example: hangSubsystem.level1Ascent();
        });
    }

    @Override
    public void initialize() {
        super.reset();

        // Build the follower from Constants and set where the robot starts on the field
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        // schedule() queues commands to run sequentially; each command finishes before the next starts
        schedule(
                // Score preload
                new FollowPathCommand(follower, scorePreload),
                openOuttakeClaw(),
                new WaitCommand(1000), // Wait 1 second

                // First pickup cycle
                new FollowPathCommand(follower, grabPickup1).setGlobalMaxPower(0.5), // Sets globalMaxPower to 50% for all future paths
                // (unless a custom maxPower is given)
                grabSample(),
                new FollowPathCommand(follower, scorePickup1),
                scoreSample(),

                // Second pickup cycle
                new FollowPathCommand(follower, grabPickup2),
                grabSample(),
                new FollowPathCommand(follower, scorePickup2, 1.0), // Overrides maxPower to 100% for this path only
                scoreSample(),

                // Third pickup cycle
                new FollowPathCommand(follower, grabPickup3),
                grabSample(),
                new FollowPathCommand(follower, scorePickup3),
                scoreSample(),

                // Park
                new FollowPathCommand(follower, park, false), // park with holdEnd false
                level1Ascent()
        );
    }

    @Override
    public void run() {
        super.run();
        // update() must be called every loop so the follower can compute and apply motor corrections
        follower.update();

        // Stream current pose to driver station telemetry each loop
        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }
}
