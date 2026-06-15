package org.firstinspires.ftc.teamcode.samples;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// CommandOpMode drives everything through a sequential/parallel command scheduler
import com.seattlesolvers.solverslib.command.CommandOpMode;
// RunCommand runs a lambda every loop tick for as long as it's scheduled
import com.seattlesolvers.solverslib.command.RunCommand;
// FollowPathCommand follows a PathChain and ends when the path is complete
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
// HoldPointCommand locks the robot to a specific Pose using all PIDFs
import com.seattlesolvers.solverslib.pedroCommand.HoldPointCommand;
// TurnCommand rotates the robot by a relative angle
import com.seattlesolvers.solverslib.pedroCommand.TurnCommand;
// TurnToCommand rotates to an absolute field heading
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Reference OpMode that demonstrates every Pedro command type — not a full auto routine
@Autonomous
public class PedroCommands extends CommandOpMode {
    Follower follower;

    // Target pose used in HoldPointCommand examples — X, Y in inches, heading in degrees
    Pose pose = new Pose(
            72, 72, 90
    );

    PathChain pathChain;

    @Override
    public void initialize() {
        super.reset();

        // Build a simple straight-line path to demonstrate FollowPathCommand
        pathChain = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(0, 0, Math.toRadians(0)),
                        new Pose(16, 28, Math.toRadians(90)))
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        schedule(
                // Updates follower to follow path
                // RunCommand keeps follower.update() running every loop tick as a background command
                new RunCommand(() -> follower.update()),

                // HoldPointCommand — locks robot to the given Pose; second arg = whether to use heading hold
                new HoldPointCommand(follower, new Pose(0, 4, 0), false),
                new HoldPointCommand(follower, pose, true),

                // TurnCommand — rotates by a relative angle (radians by default, or pass AngleUnit.DEGREES)
                new TurnCommand(follower, Math.PI / 2, false),
                new TurnCommand(follower, 90.0, true, AngleUnit.DEGREES),

                // TurnToCommand — rotates to an absolute field heading
                new TurnToCommand(follower, Math.PI / 2),
                new TurnToCommand(follower, 90.0, AngleUnit.DEGREES),

                // FollowPathCommand variations: basic, holdEnd, holdEnd+maxPower, holdEnd+maxPower+globalMaxPower
                new FollowPathCommand(follower, pathChain),
                new FollowPathCommand(follower, pathChain, true),
                new FollowPathCommand(follower, pathChain, true, 1.0),
                new FollowPathCommand(follower, pathChain, true, 1.0).setGlobalMaxPower(1.0)
        );
    }

    @Override
    public void run() {
        super.run();
    }
}
