package com.pedropathing.drivetrain;

import static com.pedropathing.drivetrain.FollowerConstants.*;

import com.pedropathing.drivetrain.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.drivetrain.util.CustomPIDFCoefficients;
import com.pedropathing.drivetrain.util.FilteredPIDFController;
import com.pedropathing.drivetrain.util.KalmanFilter;
import com.pedropathing.drivetrain.util.PIDFController;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.Pose;
import com.pedropathing.util.MathFunctions;
import com.pedropathing.util.Vector;

import java.util.Arrays;

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

    private Vector secondaryTranslationalIntegralVector;
    private Vector translationalIntegralVector;
    private Vector headingVector;
    private Vector translationalVector;
    private Vector centripetalVector;
    private Vector correctiveVector;

    private double previousSecondaryTranslationalIntegral;
    private double previousTranslationalIntegral;
    private double headingError;

    private double maxPower;
    private final double mass;
    private boolean useHeading;
    private boolean useTranslational;
    private boolean useCentripetal;

    /**
     * @param maxPower The maximum power the robot can output.
     * @param mass The mass of the robot.
     * @param useHeading Whether to use the heading PIDF controller.
     * @param useTranslational Whether to use the translational PIDF controller.
     * @param useCentripetal Whether to use the centripetal PIDF controller.
     */

    public ErrorHandler(double maxPower, double mass, boolean useHeading, boolean useTranslational, boolean useCentripetal) {
        this.maxPower = maxPower;
        this.mass = mass;
        this.useHeading = useHeading;
        this.useTranslational = useTranslational;
        this.useCentripetal = useCentripetal;

        secondaryTranslationalPIDF = new PIDFController(secondaryTranslationalPIDFCoefficients);
        secondaryTranslationalIntegral = new PIDFController(new CustomPIDFCoefficients(0, 0, 0, 0));
        translationalPIDF = new PIDFController(translationalPIDFCoefficients);
        translationalIntegral = new PIDFController(new CustomPIDFCoefficients(0, 0, 0, 0));
        secondaryHeadingPIDF = new PIDFController(secondaryHeadingPIDFCoefficients);
        headingPIDF = new PIDFController(headingPIDFCoefficients);

        secondaryTranslationalIntegralVector = new Vector();
        translationalIntegralVector = new Vector();
        headingVector = new Vector();
        translationalVector = new Vector();
        centripetalVector = new Vector();
        correctiveVector = new Vector();
        reset();
    }

    public void reset() {
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
        headingVector = new Vector();
        translationalVector = new Vector();
        centripetalVector = new Vector();
        correctiveVector = new Vector();
        headingError = 0;
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

    public Vector getCorrectiveVector(Path currentPath, Pose currentPose, Vector currentVelocity, Pose closestPose) {
        Vector centripetal = getCentripetalForceCorrection(currentPath, currentVelocity);
        Vector translational = getTranslationalCorrection(currentPath, currentPose, closestPose);
        
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

    private Vector getTranslationalCorrection(Path currentPath, Pose currentPose, Pose closestPose) {
        if (!useTranslational) return new Vector();

        Vector translationalVector = new Vector();
        double x = closestPose.getX() - currentPose.getX();
        double y = closestPose.getY() - currentPose.getY();
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
    
    public void update(boolean useHeading, boolean useTranslational, boolean useCentripetal, double maxPower) {
        this.maxPower = maxPower;
        this.useHeading = useHeading;
        this.useTranslational = useTranslational;
        this.useCentripetal = useCentripetal;
    }
}