package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    // ── Drivetrain ────────────────────────────────────────────────────────────
    public static final MecanumConstants mecanumConstants;
    static {
        mecanumConstants = new MecanumConstants()
                .leftFrontMotorName("leftFront")
                .leftRearMotorName("leftRear")
                .rightFrontMotorName("rightFront")
                .rightRearMotorName("rightRear")
                .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                .xVelocity(57.497627798966526)
                .yVelocity(49.28698874662846);

        // Recompute frontLeftVector from actual measured velocities, not defaults
        double[] polar = Pose.cartesianToPolar(mecanumConstants.xVelocity, -mecanumConstants.yVelocity);
        mecanumConstants.frontLeftVector = new Vector(polar[0], polar[1]).normalize();
    }

    // ── Pinpoint Localizer ────────────────────────────────────────────────────
    public static final PinpointConstants pinpointConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")
            .forwardPodY(-2.75)
            .strafePodX(5.5)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .yawScalar(-1.0)
            .distanceUnit(DistanceUnit.INCH);

    // ── Path Constraints ─────────────────────────────────────────────────────
    public static final PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    // ── Follower Constants ────────────────────────────────────────────────────
    public static final FollowerConstants followerConstants = new FollowerConstants();
    static {
        followerConstants.mass = 5.7;
        followerConstants.setForwardZeroPowerAcceleration(-21.605881613793525);
        followerConstants.setLateralZeroPowerAcceleration(-46.828514408535554);
    }

    public static void applyPIDF() {
        followerConstants.setCoefficientsHeadingPIDF(new PIDFCoefficients(Tuning.headingP, Tuning.headingI, Tuning.headingD, Tuning.headingF));
        followerConstants.setCoefficientsSecondaryHeadingPIDF(new PIDFCoefficients(Tuning.secondaryHeadingP, Tuning.secondaryHeadingI, Tuning.secondaryHeadingD, Tuning.secondaryHeadingF));
        followerConstants.setCoefficientsTranslationalPIDF(new PIDFCoefficients(Tuning.translationalP, Tuning.translationalI, Tuning.translationalD, Tuning.translationalF));
        followerConstants.setCoefficientsSecondaryTranslationalPIDF(new PIDFCoefficients(Tuning.secondaryTranslationalP, Tuning.secondaryTranslationalI, Tuning.secondaryTranslationalD, Tuning.secondaryTranslationalF));
        followerConstants.setCoefficientsDrivePIDF(new FilteredPIDFCoefficients(Tuning.driveP, Tuning.driveI, Tuning.driveD, Tuning.driveF, Tuning.driveFilter));
        followerConstants.setCoefficientsSecondaryDrivePIDF(new FilteredPIDFCoefficients(Tuning.secondaryDriveP, Tuning.secondaryDriveI, Tuning.secondaryDriveD, Tuning.secondaryDriveF, Tuning.secondaryDriveFilter));
        followerConstants.setCentripetalScaling(Tuning.centripetalScaling);
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
