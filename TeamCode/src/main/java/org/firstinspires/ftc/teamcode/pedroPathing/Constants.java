package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    // ── Drivetrain ────────────────────────────────────────────────────────────
    // Change motor names to match your REV Control Hub robot configuration.
    public static final MecanumConstants mecanumConstants = new MecanumConstants()
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftRear")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    // ── Localizer ─────────────────────────────────────────────────────────────
    // Using drive-encoder localizer by default (no dead wheels required).
    // Switch to threeWheelLocalizer() in createFollower() if you have dead wheels.
    public static final DriveEncoderConstants driveEncoderConstants = new DriveEncoderConstants()
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftRear")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear");

    // ── Path Constraints ─────────────────────────────────────────────────────
    public static final PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    // ── Follower Constants ────────────────────────────────────────────────────
    public static final FollowerConstants followerConstants = new FollowerConstants();

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(mecanumConstants)
                .driveEncoderLocalizer(driveEncoderConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
