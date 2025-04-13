package com.pedropathing.constants;

import static com.pedropathing.constants.ConstantsUtils.requireNotNull;
import static com.pedropathing.constants.ConstantsUtils.requireNotZero;
import static java.lang.Math.PI;

import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.Vector;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.KalmanFilterParameters;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import java.util.Objects;

public final class FollowerConstants {

    public FollowerConstants(Configurator<FollowerConstants> configurator) {
        configurator.configure(this);
        requireNotNull(leftFrontMotorName, "leftFrontMotorName");
        requireNotNull(leftRearMotorName, "leftRearMotorName");
        requireNotNull(rightFrontMotorName, "rightFrontMotorName");
        requireNotNull(rightRearMotorName, "rightRearMotorName");
        requireNotNull(leftFrontMotorDirection, "leftFrontMotorDirection");
        requireNotNull(leftRearMotorDirection, "leftRearMotorDirection");
        requireNotNull(rightFrontMotorDirection, "rightFrontMotorDirection");
        requireNotNull(rightRearMotorDirection, "rightRearMotorDirection");
        requireNotZero(mass, "mass");
    }

    // region motors
    public String leftFrontMotorName;
    public String leftRearMotorName;
    public String rightFrontMotorName;
    public String rightRearMotorName;
    public Direction leftFrontMotorDirection;
    public Direction leftRearMotorDirection;
    public Direction rightFrontMotorDirection;
    public Direction rightRearMotorDirection;
    // endregion

    // region velocities
    public double motorCachingThreshold = 0.01;
    public double xMovement = 81.34056;
    public double yMovement = 65.43028;

    private double[] getMovementAsPolar() {
        return Point.cartesianToPolar(xMovement, -yMovement);
    }

    public Vector getFrontLeftVector() {
        double[] polar = getMovementAsPolar();
        return MathFunctions.normalizeVector(new Vector(polar[0], polar[1]));
    }
    // endregion

    public double maxPower = 1;

    // region pidf coefficients
    public CustomPIDFCoefficients translationalPIDFCoefficients = new CustomPIDFCoefficients(
            0.1,
            0,
            0,
            0);
    public CustomPIDFCoefficients translationalIntegral = new CustomPIDFCoefficients(0, 0, 0, 0);
    public double translationalPIDFFeedForward = 0.015;
    public CustomPIDFCoefficients headingPIDFCoefficients = new CustomPIDFCoefficients(1, 0, 0, 0);
    public double headingPIDFFeedForward = 0.01;
    public CustomFilteredPIDFCoefficients drivePIDFCoefficients =
            new CustomFilteredPIDFCoefficients(
                    0.025,
                    0,
                    1e-5,
                    0.6,
                    0);
    public double drivePIDFFeedForward = 0.01;
    public KalmanFilterParameters driveKalmanFilterParameters = new KalmanFilterParameters(6, 1);
    // endregion

    public double mass;

    public double centripetalScaling = 0.0005;

    public double forwardZeroPowerAcceleration = -34.62719;
    public double lateralZeroPowerAcceleration = -78.15554;
    public double zeroPowerAccelerationMultiplier = 4;

    public PathEndConstraints pathEndConstraints = new PathEndConstraints(
            0.1,
            0.1,
            0.007,
            0.996,
            500
    );

    public double holdPointTranslationalScaling = 0.45;
    public double holdPointHeadingScaling = 0.35;

    // region secondary pids
    public boolean useSecondaryTranslationalPID = false;
    public boolean useSecondaryHeadingPID = false;
    public boolean useSecondaryDrivePID = false;

    public double translationalPIDFSwitch = 3;
    public CustomPIDFCoefficients secondaryTranslationalPIDFCoefficients =
            new CustomPIDFCoefficients(0.3, 0, 0.01, 0);
    public CustomPIDFCoefficients secondaryTranslationalIntegral =
            new CustomPIDFCoefficients(0, 0, 0, 0);
    public double secondaryTranslationalPIDFFeedForward = 0.015;

    public double headingPIDFSwitch = PI / 20;
    public CustomPIDFCoefficients secondaryHeadingPIDFCoefficients =
            new CustomPIDFCoefficients(5, 0, 0.08, 0);
    public double secondaryHeadingPIDFFeedForward = 0.01;
    public double drivePIDFSwitch = 20;
    public CustomFilteredPIDFCoefficients secondaryDrivePIDFCoefficients =
            new CustomFilteredPIDFCoefficients(0.02, 0, 0.000005, 0.6, 0);
    public double secondaryDrivePIDFFeedForward = 0.01;
    // endregion

    // region options
    public boolean useBrakeModeInTeleOp = false;
    public boolean automaticHoldEnd = true;
    public boolean useVoltageCompensationInAuto = false;
    public boolean useVoltageCompensationInTeleOp = false;
    // endregion

    public double nominalVoltage = 12.0;
    public double cacheInvalidateSeconds = 0.5;
    public double turnHeadingErrorThreshold = 0.01;
    public double decelerationStartMultiplier = 1;

    public int AVERAGED_VELOCITY_SAMPLE_NUMBER = 8;
    public int BEZIER_CURVE_SEARCH_LIMIT = 10;
    public int APPROXIMATION_STEPS = 1000;

}
