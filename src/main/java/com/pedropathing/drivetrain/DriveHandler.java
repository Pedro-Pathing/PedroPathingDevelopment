package com.pedropathing.drivetrain;

import static com.pedropathing.drivetrain.FollowerConstants.driveKalmanFilterParameters;
import static com.pedropathing.drivetrain.FollowerConstants.drivePIDFCoefficients;
import static com.pedropathing.drivetrain.FollowerConstants.drivePIDFFeedForward;
import static com.pedropathing.drivetrain.FollowerConstants.drivePIDFSwitch;
import static com.pedropathing.drivetrain.FollowerConstants.forwardZeroPowerAcceleration;
import static com.pedropathing.drivetrain.FollowerConstants.lateralZeroPowerAcceleration;
import static com.pedropathing.drivetrain.FollowerConstants.secondaryDrivePIDFCoefficients;
import static com.pedropathing.drivetrain.FollowerConstants.secondaryDrivePIDFFeedForward;
import static com.pedropathing.drivetrain.FollowerConstants.useSecondaryDrivePID;

import com.pedropathing.drivetrain.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.drivetrain.util.FilteredPIDFController;
import com.pedropathing.drivetrain.util.KalmanFilter;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Pose;
import com.pedropathing.util.MathFunctions;
import com.pedropathing.util.Vector;

import java.util.Arrays;

public class DriveHandler {
    private FilteredPIDFController secondaryDrivePIDF;
    private FilteredPIDFController drivePIDF;
    private boolean useDrive;
    private double maxPower;
    private double[] driveErrors;
    private double driveError;
    private double rawDriveError;
    private double previousRawDriveError;
    private Vector driveVector;
    private KalmanFilter driveKalmanFilter;

    public DriveHandler(boolean useDrive, double maxPower) {
        secondaryDrivePIDF = new FilteredPIDFController(secondaryDrivePIDFCoefficients);
        drivePIDF = new FilteredPIDFController(drivePIDFCoefficients);
        driveKalmanFilter = new KalmanFilter(driveKalmanFilterParameters);
        this.useDrive = useDrive;
        this.maxPower = maxPower;
        driveVector = new Vector();
        driveErrors = new double[2];
    }

    public void reset() {
        rawDriveError = 0;
        previousRawDriveError = 0;
        Arrays.fill(driveErrors, 0);
        driveKalmanFilter.reset();
        driveError = 0;
        driveVector = new Vector();
        secondaryDrivePIDF.reset();
        drivePIDF.reset();
    }

    /**
     * This will update the PIDF coefficients for secondary Drive PIDF mid run
     * can be used between paths
     *
     * @param set PIDF coefficients you would like to set.
     */
    public void setSecondaryDrivePIDF(CustomFilteredPIDFCoefficients set){
        secondaryDrivePIDF.setCoefficients(set);
    }

    public void update(boolean useDrive, double maxPower) {
        this.useDrive = useDrive;
        this.maxPower = maxPower;
    }

    public double getDriveError() {
        return driveError;
    }

    /**
     * This will update the PIDF coefficients for primary Drive PIDF mid run
     * can be used between paths
     *
     * @param set PIDF coefficients you would like to set.
     */
    public void setDrivePIDF(CustomFilteredPIDFCoefficients set){
        drivePIDF.setCoefficients(set);
    }

    private double getDriveVelocityError(Path currentPath, Pose currentPose, Vector currentVelocity, double distanceToGoal) {
        Vector distanceToGoalVector = MathFunctions.scalarMultiplyVector(
                MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector()), distanceToGoal);
        Vector velocity = new Vector(MathFunctions.dotProduct(currentVelocity,
                MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())),
                currentPath.getClosestPointTangentVector().getTheta());

        Vector forwardHeadingVector = new Vector(1.0, currentPose.getHeading());
        double forwardVelocity = MathFunctions.dotProduct(forwardHeadingVector, velocity);
        double forwardDistanceToGoal = MathFunctions.dotProduct(forwardHeadingVector, distanceToGoalVector);
        double forwardVelocityGoal = MathFunctions.getSign(forwardDistanceToGoal) *
                Math.sqrt(Math.abs(-2 * currentPath.getZeroPowerAccelerationMultiplier() *
                        forwardZeroPowerAcceleration * (forwardDistanceToGoal <= 0 ? 1 : -1) * forwardDistanceToGoal));
        double forwardVelocityZeroPowerDecay = forwardVelocity - MathFunctions.getSign(forwardDistanceToGoal) *
                Math.sqrt(Math.abs(Math.pow(forwardVelocity, 2) + 2 * forwardZeroPowerAcceleration * Math.abs(forwardDistanceToGoal)));

        Vector lateralHeadingVector = new Vector(1.0, currentPose.getHeading() - Math.PI / 2);
        double lateralVelocity = MathFunctions.dotProduct(lateralHeadingVector, velocity);
        double lateralDistanceToGoal = MathFunctions.dotProduct(lateralHeadingVector, distanceToGoalVector);
        double lateralVelocityGoal = MathFunctions.getSign(lateralDistanceToGoal) *
                Math.sqrt(Math.abs(-2 * currentPath.getZeroPowerAccelerationMultiplier() *
                        lateralZeroPowerAcceleration * (lateralDistanceToGoal <= 0 ? 1 : -1) * lateralDistanceToGoal));
        double lateralVelocityZeroPowerDecay = lateralVelocity - MathFunctions.getSign(lateralDistanceToGoal) *
                Math.sqrt(Math.abs(Math.pow(lateralVelocity, 2) + 2 * lateralZeroPowerAcceleration * Math.abs(lateralDistanceToGoal)));

        Vector forwardVelocityError = new Vector(forwardVelocityGoal - forwardVelocityZeroPowerDecay - forwardVelocity,
                forwardHeadingVector.getTheta());
        Vector lateralVelocityError = new Vector(lateralVelocityGoal - lateralVelocityZeroPowerDecay - lateralVelocity,
                lateralHeadingVector.getTheta());
        Vector velocityErrorVector = MathFunctions.addVectors(forwardVelocityError, lateralVelocityError);

        previousRawDriveError = rawDriveError;
        rawDriveError = velocityErrorVector.getMagnitude() *
                MathFunctions.getSign(MathFunctions.dotProduct(velocityErrorVector, currentPath.getClosestPointTangentVector()));

        double projection = 2 * driveErrors[1] - driveErrors[0];
        driveKalmanFilter.update(rawDriveError - previousRawDriveError, projection);

        for (int i = 0; i < driveErrors.length - 1; i++) {
            driveErrors[i] = driveErrors[i + 1];
        }
        driveErrors[1] = driveKalmanFilter.getState();

        return driveKalmanFilter.getState();
    }

    public Vector getDriveVector(Path currentPath, Pose currentPose, Vector currentVelocity, double distanceToGoal) {
        if (!useDrive) return new Vector();

        driveError = getDriveVelocityError(currentPath, currentPose, currentVelocity, distanceToGoal);

        if (Math.abs(driveError) < drivePIDFSwitch && useSecondaryDrivePID) {
            secondaryDrivePIDF.updateError(driveError);
            driveVector = new Vector(MathFunctions.clamp(secondaryDrivePIDF.runPIDF() + secondaryDrivePIDFFeedForward * MathFunctions.getSign(driveError), -maxPower, maxPower),
                    currentPath.getClosestPointTangentVector().getTheta());
            return MathFunctions.copyVector(driveVector);
        }

        drivePIDF.updateError(driveError);
        driveVector = new Vector(MathFunctions.clamp(drivePIDF.runPIDF() + drivePIDFFeedForward * MathFunctions.getSign(driveError), -maxPower, maxPower),
                currentPath.getClosestPointTangentVector().getTheta());
        return MathFunctions.copyVector(driveVector);
    }
}
