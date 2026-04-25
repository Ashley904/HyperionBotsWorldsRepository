package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
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
import org.firstinspires.ftc.teamcode.util.rConstants;

@Config
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(15.5)
            .forwardZeroPowerAcceleration(-47.895953908499635)
            .lateralZeroPowerAcceleration(-74.7123052346908)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.23,
                    0.0,
                    0.02,
                    0.02
            ))
            .useSecondaryTranslationalPIDF(false)
            .translationalPIDFSwitch(4)
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1.4,
                    0.0,
                    0.03,
                    0.03
            ))
            .useSecondaryHeadingPIDF(false)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.0065,
                    0.0,
                    0.0001,
                    0.6,
                    0.0775
            ))
            .useSecondaryDrivePIDF(false)
            .drivePIDFSwitch(15)
            .centripetalScaling(0.0005);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("FL")
            .leftRearMotorName("BL")
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(70.529094554585998)
            .yVelocity(52.173685389240894);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-7.5)
            .strafePodX(-0.79)
            .distanceUnit(DistanceUnit.INCH)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    /**
     These are the PathConstraints in order:
     tValueConstraint, velocityConstraint, translationalConstraint, headingConstraint, timeoutConstraint,
     brakingStrength, BEZIER_CURVE_SEARCH_LIMIT, brakingStart

     The BEZIER_CURVE_SEARCH_LIMIT should typically be left at 10 and shouldn't be changed.
     */

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            1.25,
            10,
            1
    );

    //Add custom localizers or drivetrains here
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}