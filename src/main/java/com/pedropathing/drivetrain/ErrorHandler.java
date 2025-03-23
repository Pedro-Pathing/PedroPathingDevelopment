package com.pedropathing.drivetrain;

import static com.pedropathing.drivetrain.FollowerConstants.*;

import com.pedropathing.drivetrain.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.drivetrain.util.CustomPIDFCoefficients;
import com.pedropathing.drivetrain.util.FilteredPIDFController;
import com.pedropathing.drivetrain.util.KalmanFilter;
import com.pedropathing.drivetrain.util.PIDFController;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Pose;
import com.pedropathing.util.MathFunctions;
import com.pedropathing.util.Vector;

/**
 * This is the ErrorHandler class. This class handles the error correction for the drivetrain.
 * This class is responsible for calculating the error in the drivetrain's position and heading
 * and outputting the corrective vectors.
 *
 * @version 1.0, 3/23/2025
 * @author Baron Henderson - 20077 The Indubitables
 */
public class ErrorHandler {
    private PIDFController secondaryTranslationalPIDF;
    private PIDFController secondaryTranslationalIntegral;
    private PIDFController translationalPIDF;
    private PIDFController translationalIntegral;
    private PIDFController secondaryHeadingPIDF;
    private PIDFController headingPIDF;
    private FilteredPIDFController secondaryDrivePIDF;
    private FilteredPIDFController drivePIDF;
    private KalmanFilter driveKalmanFilter;

    private Vector secondaryTranslationalIntegralVector;
    private Vector translationalIntegralVector;
    private Vector driveVector;
    private Vector headingVector;
    private Vector translationalVector;
    private Vector centripetalVector;
    private Vector correctiveVector;

    private double previousSecondaryTranslationalIntegral;
    private double previousTranslationalIntegral;
    private double driveError;
    private double headingError;
    private double rawDriveError;
    private double previousRawDriveError;
    private double[] driveErrors;

    private double maxPower;
    private double mass;
    private boolean useDrive;
    private boolean useHeading;
    private boolean useTranslational;
    private boolean useCentripetal;

    /**
     * @param maxPower The maximum power the robot can output.
     * @param mass The mass of the robot.
     * @param useDrive Whether to use the drive PIDF controller.
     * @param useHeading Whether to use the heading PIDF controller.
     * @param useTranslational Whether to use the translational PIDF controller.
     * @param useCentripetal Whether to use the centripetal PIDF controller.
     */

    public ErrorHandler(double maxPower, double mass, boolean useDrive, boolean useHeading, boolean useTranslational, boolean useCentripetal) {
        this.maxPower = maxPower;
        this.mass = mass;
        this.useDrive = useDrive;
        this.useHeading = useHeading;
        this.useTranslational = useTranslational;
        this.useCentripetal = useCentripetal;

        secondaryTranslationalPIDF = new PIDFController(secondaryTranslationalPIDFCoefficients);
        secondaryTranslationalIntegral = new PIDFController(new CustomPIDFCoefficients(0, 0, 0, 0));
        translationalPIDF = new PIDFController(translationalPIDFCoefficients);
        translationalIntegral = new PIDFController(new CustomPIDFCoefficients(0, 0, 0, 0));
        secondaryHeadingPIDF = new PIDFController(secondaryHeadingPIDFCoefficients);
        headingPIDF = new PIDFController(headingPIDFCoefficients);
        secondaryDrivePIDF = new FilteredPIDFController(secondaryDrivePIDFCoefficients);
        drivePIDF = new FilteredPIDFController(drivePIDFCoefficients);
        driveKalmanFilter = new KalmanFilter(driveKalmanFilterParameters);

        secondaryTranslationalIntegralVector = new Vector();
        translationalIntegralVector = new Vector();
        driveVector = new Vector();
        headingVector = new Vector();
        translationalVector = new Vector();
        centripetalVector = new Vector();
        correctiveVector = new Vector();

        driveErrors = new double[2];
        reset();
    }

    public void reset() {
        secondaryDrivePIDF.reset();
        drivePIDF.reset();
        secondaryHeadingPIDF.reset();
        headingPIDF.reset();
        secondaryTranslationalPIDF.reset();
        secondaryTranslationalIntegral.reset();
        secondaryTranslationalIntegralVector = new Vector();
        previousSecondaryTranslationalIntegral = 0;
        translationalPIDF.reset();
        translationalIntegral.reset();
        translationalIntegralVector = new Vector();
        previousTranslationalIntegral = 0;
        driveVector = new Vector();
        headingVector = new Vector();
        translationalVector = new Vector();
        centripetalVector = new Vector();
        correctiveVector = new Vector();
        driveError = 0;
        headingError = 0;
        rawDriveError = 0;
        previousRawDriveError = 0;
        for (int i = 0; i < driveErrors.length; i++) {
            driveErrors[i] = 0;
        }
        driveKalmanFilter.reset();
    }

    public Vector getDriveVector(Path currentPath, Pose currentPose, Vector currentVelocity) {
        if (!useDrive) return new Vector();

        driveError = getDriveVelocityError(currentPath, currentPose, currentVelocity);

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

    public Vector getHeadingVector(Path currentPath, Pose currentPose) {
        if (!useHeading) return new Vector();
        
        headingError = MathFunctions.getTurnDirection(currentPose.getHeading(), currentPath.getClosestPointHeadingGoal()) * 
                      MathFunctions.getSmallestAngleDifference(currentPose.getHeading(), currentPath.getClosestPointHeadingGoal());
        
        if (Math.abs(headingError) < headingPIDFSwitch && useSecondaryHeadingPID) {
            secondaryHeadingPIDF.updateError(headingError);
            headingVector = new Vector(MathFunctions.clamp(secondaryHeadingPIDF.runPIDF() + secondaryHeadingPIDFFeedForward * 
                                                         MathFunctions.getTurnDirection(currentPose.getHeading(), currentPath.getClosestPointHeadingGoal()), 
                                                         -maxPower, maxPower), currentPose.getHeading());
            return MathFunctions.copyVector(headingVector);
        }
        
        headingPIDF.updateError(headingError);
        headingVector = new Vector(MathFunctions.clamp(headingPIDF.runPIDF() + headingPIDFFeedForward * 
                                                     MathFunctions.getTurnDirection(currentPose.getHeading(), currentPath.getClosestPointHeadingGoal()), 
                                                     -maxPower, maxPower), currentPose.getHeading());
        return MathFunctions.copyVector(headingVector);
    }

    public Vector getCorrectiveVector(Path currentPath, Pose currentPose, Vector currentVelocity) {
        Vector centripetal = getCentripetalForceCorrection(currentPath, currentVelocity);
        Vector translational = getTranslationalCorrection(currentPath, currentPose);
        
        // Combine vectors and normalize if needed
        Vector corrective = MathFunctions.addVectors(centripetal, translational);
        if (corrective.getMagnitude() > maxPower) {
            double scalingFactor = Math.min(
                findNormalizingScaling(centripetal, translational),
                findNormalizingScaling(translational, centripetal)
            );
            corrective = MathFunctions.addVectors(
                centripetal,
                MathFunctions.scalarMultiplyVector(translational, scalingFactor)
            );
        }

        correctiveVector = MathFunctions.copyVector(corrective);
        return corrective;
    }

    private double findNormalizingScaling(Vector staticVector, Vector variableVector) {
        double a = Math.pow(variableVector.getXComponent(), 2) + Math.pow(variableVector.getYComponent(), 2);
        double b = staticVector.getXComponent() * variableVector.getXComponent() + 
                   staticVector.getYComponent() * variableVector.getYComponent();
        double c = Math.pow(staticVector.getXComponent(), 2) + 
                   Math.pow(staticVector.getYComponent(), 2) - 
                   Math.pow(maxPower, 2);
        return (-b + Math.sqrt(Math.pow(b, 2) - a*c))/(a);
    }

    private double getDriveVelocityError(Path currentPath, Pose currentPose, Vector currentVelocity) {
        double distanceToGoal;
        if (!currentPath.isAtParametricEnd()) {
            distanceToGoal = currentPath.length() * (1 - currentPath.getClosestPointTValue());
        } else {
            Vector offset = new Vector();
            offset.setOrthogonalComponents(currentPose.getX() - currentPath.getLastControlPoint().getX(), 
                                         currentPose.getY() - currentPath.getLastControlPoint().getY());
            distanceToGoal = MathFunctions.dotProduct(currentPath.getEndTangent(), offset);
        }

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

    private Vector getTranslationalCorrection(Path currentPath, Pose currentPose) {
        if (!useTranslational) return new Vector();
        
        Vector translationalVector = new Vector();
        double x = currentPose.getX() - currentPose.getX();
        double y = currentPose.getY() - currentPose.getY();
        translationalVector.setOrthogonalComponents(x, y);

        if (!(currentPath.isAtParametricEnd() || currentPath.isAtParametricStart())) {
            translationalVector = MathFunctions.subtractVectors(translationalVector, 
                new Vector(MathFunctions.dotProduct(translationalVector, 
                    MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), 
                    currentPath.getClosestPointTangentVector().getTheta()));

            secondaryTranslationalIntegralVector = MathFunctions.subtractVectors(secondaryTranslationalIntegralVector, 
                new Vector(MathFunctions.dotProduct(secondaryTranslationalIntegralVector, 
                    MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), 
                    currentPath.getClosestPointTangentVector().getTheta()));
            translationalIntegralVector = MathFunctions.subtractVectors(translationalIntegralVector, 
                new Vector(MathFunctions.dotProduct(translationalIntegralVector, 
                    MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), 
                    currentPath.getClosestPointTangentVector().getTheta()));
        }

        if (MathFunctions.distance(currentPose, currentPath.getClosestPoint(currentPose, 1)) < translationalPIDFSwitch &&
            useSecondaryTranslationalPID) {
            secondaryTranslationalIntegral.updateError(translationalVector.getMagnitude());
            secondaryTranslationalIntegralVector = MathFunctions.addVectors(secondaryTranslationalIntegralVector, 
                new Vector(secondaryTranslationalIntegral.runPIDF() - previousSecondaryTranslationalIntegral, 
                    translationalVector.getTheta()));
            previousSecondaryTranslationalIntegral = secondaryTranslationalIntegral.runPIDF();

            secondaryTranslationalPIDF.updateError(translationalVector.getMagnitude());
            translationalVector.setMagnitude(secondaryTranslationalPIDF.runPIDF() + secondaryTranslationalPIDFFeedForward);
            translationalVector = MathFunctions.addVectors(translationalVector, secondaryTranslationalIntegralVector);
        } else {
            translationalIntegral.updateError(translationalVector.getMagnitude());
            translationalIntegralVector = MathFunctions.addVectors(translationalIntegralVector, 
                new Vector(translationalIntegral.runPIDF() - previousTranslationalIntegral, 
                    translationalVector.getTheta()));
            previousTranslationalIntegral = translationalIntegral.runPIDF();

            translationalPIDF.updateError(translationalVector.getMagnitude());
            translationalVector.setMagnitude(translationalPIDF.runPIDF() + translationalPIDFFeedForward);
            translationalVector = MathFunctions.addVectors(translationalVector, translationalIntegralVector);
        }

        translationalVector.setMagnitude(MathFunctions.clamp(translationalVector.getMagnitude(), 0, maxPower));
        this.translationalVector = MathFunctions.copyVector(translationalVector);
        return translationalVector;
    }

    private Vector getCentripetalForceCorrection(Path currentPath, Vector currentVelocity) {
        if (!useCentripetal) return new Vector();
        
        double curvature = currentPath.getClosestPointCurvature();
        if (Double.isNaN(curvature)) return new Vector();
        
        centripetalVector = new Vector(MathFunctions.clamp(centripetalScaling * mass * 
            Math.pow(MathFunctions.dotProduct(currentVelocity, 
                MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), 2) * curvature, 
            -maxPower, maxPower), 
            currentPath.getClosestPointTangentVector().getTheta() + Math.PI / 2 * 
            MathFunctions.getSign(currentPath.getClosestPointNormalVector().getTheta()));
        return centripetalVector;
    }

    public double getHeadingError() {
        return headingError;
    }

    public double getDriveError() {
        return driveError;
    }

    /**
     * This will update the PIDF coefficients for primary Heading PIDF mid run
     * can be used between paths
     *
     * @param set PIDF coefficients you would like to set.
     */
    public void setHeadingPIDF(CustomPIDFCoefficients set){
        headingPIDF.setCoefficients(set);
    }

    /**
     * This will update the PIDF coefficients for primary Translational PIDF mid run
     * can be used between paths
     *
     * @param set PIDF coefficients you would like to set.
     */
    public void setTranslationalPIDF(CustomPIDFCoefficients set){
        translationalPIDF.setCoefficients(set);
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

    /**
     * This will update the PIDF coefficients for secondary Heading PIDF mid run
     * can be used between paths
     *
     * @param set PIDF coefficients you would like to set.
     */
    public void setSecondaryHeadingPIDF(CustomPIDFCoefficients set){
        secondaryHeadingPIDF.setCoefficients(set);
    }

    /**
     * This will update the PIDF coefficients for secondary Translational PIDF mid run
     * can be used between paths
     *
     * @param set PIDF coefficients you would like to set.
     */
    public void setSecondaryTranslationalPIDF(CustomPIDFCoefficients set){
        secondaryTranslationalPIDF.setCoefficients(set);
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
    
    public void update(boolean useDrive, boolean useHeading, boolean useTranslational, boolean useCentripetal, double maxPower, double mass) {
        this.maxPower = maxPower;
        this.useDrive = useDrive;
        this.useHeading = useHeading;
        this.useTranslational = useTranslational;
        this.useCentripetal = useCentripetal;
        this.mass = mass;
    }
}
