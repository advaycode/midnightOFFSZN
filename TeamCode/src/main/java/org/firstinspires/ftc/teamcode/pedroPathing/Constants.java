package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants {

    // ── Drivetrain ────────────────────────────────────────────────────────────
    public static final MecanumConstants mecanumConstants = new MecanumConstants()
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftRear")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(57.497627798966526)
            .yVelocity(49.28698874662846);

    // ── Pinpoint Localizer ────────────────────────────────────────────────────
    // hardwareMapName: name it "pinpoint" in Driver Hub config, I2C port 1-3 (NOT 0)
    // forwardPodY:  inches from robot center → positive = forward of center
    // strafePodX:  inches from robot center → positive = left of center
    // Measure both with a ruler from the center of the robot to each pod.
    public static final PinpointConstants pinpointConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")
            .forwardPodY(-2.75)   // TODO: measure and fill in (inches)
            .strafePodX(5.5)    // TODO: measure and fill in (inches)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .yawScalar(-1.0)
            .distanceUnit(DistanceUnit.INCH);

    // ── Path Constraints ─────────────────────────────────────────────────────
    public static final PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    // ── PIDF values — editable live from Panels dashboard ────────────────────
    // Primary heading
    public static double headingP = 4.0;
    public static double headingI = 0.0;
    public static double headingD = 0.05;
    public static double headingF = 0.0;

    // Secondary heading (kicks in when error < headingPIDFSwitch)
    public static double secondaryHeadingP = 2.0;
    public static double secondaryHeadingI = 0.0;
    public static double secondaryHeadingD = 0.02;
    public static double secondaryHeadingF = 0.0;

    // Primary translational
    public static double translationalP = 2.0;
    public static double translationalI = 0.0;
    public static double translationalD = 0.15;
    public static double translationalF = 0.0;

    // Secondary translational (kicks in when error < translationalPIDFSwitch)
    public static double secondaryTranslationalP = 1.0;
    public static double secondaryTranslationalI = 0.0;
    public static double secondaryTranslationalD = 0.1;
    public static double secondaryTranslationalF = 0.0;

    // Primary drive (FilteredPIDFCoefficients: P, I, D, F, filter)
    public static double driveP = 0.1;
    public static double driveI = 0.0;
    public static double driveD = 0.00035;
    public static double driveF = 0.6;
    public static double driveFilter = 0.015;

    // Secondary drive
    public static double secondaryDriveP = 0.02;
    public static double secondaryDriveI = 0.0;
    public static double secondaryDriveD = 0.000005;
    public static double secondaryDriveF = 0.6;
    public static double secondaryDriveFilter = 0.01;

    public static double centripetalScaling = 0.0005;

    // ── Follower Constants ────────────────────────────────────────────────────
    public static final FollowerConstants followerConstants = new FollowerConstants();
    static {
        followerConstants.mass = 5.7;
        followerConstants.setForwardZeroPowerAcceleration(-21.605881613793525);
        followerConstants.setLateralZeroPowerAcceleration(-46.828514408535554);
    }

    public static void applyPIDF() {
        followerConstants.setCoefficientsHeadingPIDF(new PIDFCoefficients(headingP, headingI, headingD, headingF));
        followerConstants.setCoefficientsSecondaryHeadingPIDF(new PIDFCoefficients(secondaryHeadingP, secondaryHeadingI, secondaryHeadingD, secondaryHeadingF));
        followerConstants.setCoefficientsTranslationalPIDF(new PIDFCoefficients(translationalP, translationalI, translationalD, translationalF));
        followerConstants.setCoefficientsSecondaryTranslationalPIDF(new PIDFCoefficients(secondaryTranslationalP, secondaryTranslationalI, secondaryTranslationalD, secondaryTranslationalF));
        followerConstants.setCoefficientsDrivePIDF(new FilteredPIDFCoefficients(driveP, driveI, driveD, driveF, driveFilter));
        followerConstants.setCoefficientsSecondaryDrivePIDF(new FilteredPIDFCoefficients(secondaryDriveP, secondaryDriveI, secondaryDriveD, secondaryDriveF, secondaryDriveFilter));
        followerConstants.setCentripetalScaling(centripetalScaling);
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
