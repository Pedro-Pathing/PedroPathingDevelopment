package com.pedropathing.drivetrain;

import static com.pedropathing.drivetrain.FollowerConstants.cacheInvalidateSeconds;
import static com.pedropathing.drivetrain.FollowerConstants.nominalVoltage;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.drivetrain.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.drivetrain.util.CustomPIDFCoefficients;
import com.pedropathing.localization.Tracker;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathCallback;
import com.pedropathing.util.MathFunctions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.localization.Localizer;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Pose;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Vector;
import com.pedropathing.util.Drawing;
import com.pedropathing.util.DashboardPoseTracker;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is the Follower class. This class handles the following of paths and holding of positions.
 * This class is used to follow paths and hold positions. It uses a Tracker to track the robot's
 * position, the ErrorHandler to handle the error correction and output the corrective vectors, and a Drivetrain to move the robot accordingly.
 *
 * @see Tracker
 * @see ErrorHandler
 * @see Drivetrain
 *
 * @version 1.0, 3/23/2025
 * @author Baron Henderson - 20077 The Indubitables
 */
@Config
public class Follower {
    private Drivetrain drivetrain;
    private ErrorHandler errorHandler;
    private Tracker tracker;
    private DashboardPoseTracker dashboardPoseTracker;

    private Path currentPath;
    private PathChain currentPathChain;
    private Pose closestPose;
    private int chainIndex;
    private long[] pathStartTimes;
    private boolean followingPathChain;
    private boolean holdingPosition;
    private boolean isBusy;
    private boolean isTurning;
    private boolean reachedParametricPathEnd;
    private boolean holdPositionAtEnd;
    private boolean teleopDrive;
    private long reachedParametricPathEndTime;
    private double[] teleopDriveValues;
    private Vector teleopDriveVector;
    private Vector teleopHeadingVector;

    public static boolean useDrive = true;
    public static boolean useHeading = true;
    public static boolean useTranslational = true;
    public static boolean useCentripetal = true;
    public static boolean drawOnDashboard = true;


    public Follower(HardwareMap hardwareMap, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.errorHandler = new ErrorHandler(FollowerConstants.maxPower, FollowerConstants.mass, useDrive, useHeading, useTranslational, useCentripetal);
        this.tracker = new Tracker(hardwareMap);
        this.dashboardPoseTracker = new DashboardPoseTracker(tracker);
        
        teleopDriveValues = new double[3];
        teleopDriveVector = new Vector();
        teleopHeadingVector = new Vector();
        
        drivetrain.initialize();
        breakFollowing();
    }

    public Follower(HardwareMap hardwareMap, Drivetrain drivetrain, Localizer localizer) {
        this.drivetrain = drivetrain;
        this.errorHandler = new ErrorHandler(FollowerConstants.maxPower, FollowerConstants.mass, useDrive, useHeading, useTranslational, useCentripetal);
        this.tracker = new Tracker(hardwareMap, localizer);
        this.dashboardPoseTracker = new DashboardPoseTracker(tracker);
        
        teleopDriveValues = new double[3];
        teleopDriveVector = new Vector();
        teleopHeadingVector = new Vector();
        
        drivetrain.initialize();
        breakFollowing();
    }

    /**
     * This holds a Point.
     *
     * @param point the Point to stay at.
     */
    public void holdPoint(BezierPoint point, double heading) {
        breakFollowing();
        holdingPosition = true;
        isBusy = false;
        followingPathChain = false;
        currentPath = new Path(point);
        currentPath.setConstantHeadingInterpolation(heading);
        closestPose = currentPath.getClosestPoint(tracker.getPose(), 1);
    }

    /**
     * This holds a Point.
     *
     * @param point   the Point to stay at.
     * @param heading the heading to face.
     */
    public void holdPoint(Point point, double heading) {
        holdPoint(new BezierPoint(point), heading);
    }

    /**
     * This holds a Point.
     *
     * @param pose the Point (as a Pose) to stay at.
     */
    public void holdPoint(Pose pose) {
        holdPoint(new Point(pose), pose.getHeading());
    }

    public void followPath(Path path, boolean holdEnd) {
        drivetrain.setMaxPower(FollowerConstants.maxPower);
        breakFollowing();
        holdPositionAtEnd = holdEnd;
        isBusy = true;
        followingPathChain = false;
        currentPath = path;
        closestPose = currentPath.getClosestPoint(tracker.getPose(), FollowerConstants.BEZIER_CURVE_SEARCH_LIMIT);
    }

    public void followPath(Path path) {
        followPath(path, FollowerConstants.automaticHoldEnd);
    }

    public void followPath(PathChain pathChain, boolean holdEnd) {
        followPath(pathChain, FollowerConstants.maxPower, holdEnd);
    }

    public void followPath(PathChain pathChain) {
        followPath(pathChain, FollowerConstants.automaticHoldEnd);
    }

    public void followPath(PathChain pathChain, double maxPower, boolean holdEnd) {
        drivetrain.setMaxPower(maxPower);
        breakFollowing();
        holdPositionAtEnd = holdEnd;
        pathStartTimes = new long[pathChain.size()];
        pathStartTimes[0] = System.currentTimeMillis();
        isBusy = true;
        followingPathChain = true;
        chainIndex = 0;
        currentPathChain = pathChain;
        currentPath = pathChain.getPath(chainIndex);
        closestPose = currentPath.getClosestPoint(tracker.getPose(), FollowerConstants.BEZIER_CURVE_SEARCH_LIMIT);
        currentPathChain.resetCallbacks();
    }

    /**
     * This checks if any PathCallbacks should be run right now, and runs them if applicable.
     */
    public void updateCallbacks() {
        for (PathCallback callback : currentPathChain.getCallbacks()) {
            if (!callback.hasBeenRun()) {
                if (callback.getType() == PathCallback.PARAMETRIC) {
                    // parametric call back
                    if (chainIndex == callback.getIndex() && (getCurrentTValue() >= callback.getStartCondition() || MathFunctions.roughlyEquals(getCurrentTValue(), callback.getStartCondition()))) {
                        callback.run();
                    }
                } else {
                    // time based call back
                    if (chainIndex >= callback.getIndex() && System.currentTimeMillis() - pathStartTimes[callback.getIndex()] > callback.getStartCondition()) {
                        callback.run();
                    }

                }
            }
        }
    }

    public void update() {
        tracker.update();
        errorHandler.update(useDrive, useHeading, useTranslational, useCentripetal, FollowerConstants.maxPower, FollowerConstants.mass);
        if (drawOnDashboard) {
            dashboardPoseTracker.update();
        }

        if (!teleopDrive) {
            if (currentPath != null) {
                if (holdingPosition) {
                    closestPose = currentPath.getClosestPoint(tracker.getPose(), 1);
                    Vector corrective = errorHandler.getCorrectiveVector(currentPath, tracker.getPose(), tracker.getVelocity());
                    Vector heading = errorHandler.getHeadingVector(currentPath, tracker.getPose());
                    
                    drivetrain.setDrivePowers(drivetrain.getDrivePowers(
                        MathFunctions.scalarMultiplyVector(corrective, FollowerConstants.holdPointTranslationalScaling),
                        MathFunctions.scalarMultiplyVector(heading, FollowerConstants.holdPointHeadingScaling),
                        new Vector(),
                        tracker.getPose().getHeading()
                    ));

                    if (errorHandler.getHeadingError() < FollowerConstants.turnHeadingErrorThreshold && isTurning) {
                        isTurning = false;
                        isBusy = false;
                    }
                } else {
                    if (isBusy) {
                        closestPose = currentPath.getClosestPoint(tracker.getPose(), FollowerConstants.BEZIER_CURVE_SEARCH_LIMIT);

                        if (followingPathChain) {
                            updateCallbacks();
                        }

                        Vector corrective = errorHandler.getCorrectiveVector(currentPath, tracker.getPose(), tracker.getVelocity());
                        Vector heading = errorHandler.getHeadingVector(currentPath, tracker.getPose());
                        Vector drive = errorHandler.getDriveVector(currentPath, tracker.getPose(), tracker.getVelocity());

                        drivetrain.setDrivePowers(drivetrain.getDrivePowers(
                            corrective,
                            heading,
                            drive,
                            tracker.getPose().getHeading()
                        ));

                        if (tracker.getVelocity().getMagnitude() < 1.0 && currentPath.getClosestPointTValue() > 0.8) {
                            if (System.currentTimeMillis() - reachedParametricPathEndTime > 500.0) {
                                if (followingPathChain && chainIndex < currentPathChain.size() - 1) {
                                    breakFollowing();
                                    pathStartTimes[chainIndex] = System.currentTimeMillis();
                                    isBusy = true;
                                    followingPathChain = true;
                                    chainIndex++;
                                    currentPath = currentPathChain.getPath(chainIndex);
                                    closestPose = currentPath.getClosestPoint(tracker.getPose(), FollowerConstants.BEZIER_CURVE_SEARCH_LIMIT);
                                } else {
                                    if (!reachedParametricPathEnd) {
                                        reachedParametricPathEnd = true;
                                        reachedParametricPathEndTime = System.currentTimeMillis();
                                    }

                                    if ((System.currentTimeMillis() - reachedParametricPathEndTime > currentPath.getPathEndTimeoutConstraint()) ||
                                        (tracker.getVelocity().getMagnitude() < currentPath.getPathEndVelocityConstraint() &&
                                         MathFunctions.distance(tracker.getPose(), closestPose) < currentPath.getPathEndTranslationalConstraint() &&
                                         MathFunctions.getSmallestAngleDifference(tracker.getPose().getHeading(), currentPath.getClosestPointHeadingGoal()) < currentPath.getPathEndHeadingConstraint())) {
                                        if (holdPositionAtEnd) {
                                            holdPositionAtEnd = false;
                                            holdPoint(currentPath.getLastControlPoint(), currentPath.getHeadingGoal(1));
                                        } else {
                                            breakFollowing();
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } else {
            drivetrain.setDrivePowers(drivetrain.getDrivePowers(
                new Vector(),
                teleopHeadingVector,
                teleopDriveVector,
                tracker.getPose().getHeading()
            ));
        }
    }

    public void breakFollowing() {
        teleopDrive = false;
        drivetrain.setMotorsToFloat();
        holdingPosition = false;
        isBusy = false;
        reachedParametricPathEnd = false;
        errorHandler.reset();
        drivetrain.stop();
    }

    public void startTeleopDrive() {
        breakFollowing();
        teleopDrive = true;

        if (FollowerConstants.useBrakeModeInTeleOp) {
            drivetrain.setMotorsToBrake();
        }
    }

    public void setTeleOpMovementVectors(double forwardDrive, double lateralDrive, double heading) {
        setTeleOpMovementVectors(forwardDrive, lateralDrive, heading, true);
    }

    public void setTeleOpMovementVectors(double forwardDrive, double lateralDrive, double heading, boolean robotCentric) {
        teleopDriveValues[0] = MathFunctions.clamp(forwardDrive, -1, 1);
        teleopDriveValues[1] = MathFunctions.clamp(lateralDrive, -1, 1);
        teleopDriveValues[2] = MathFunctions.clamp(heading, -1, 1);
        teleopDriveVector.setOrthogonalComponents(teleopDriveValues[0], teleopDriveValues[1]);
        teleopDriveVector.setMagnitude(MathFunctions.clamp(teleopDriveVector.getMagnitude(), 0, 1));

        if (robotCentric) {
            teleopDriveVector.rotateVector(getPose().getHeading());
        }

        teleopHeadingVector.setComponents(teleopDriveValues[2], getPose().getHeading());
    }

    public void turn(double radians, boolean isLeft) {
        Pose temp = new Pose(getPose().getX(), getPose().getY(), getPose().getHeading() + (isLeft ? radians : -radians));
        holdPoint(temp);
        isTurning = true;
        isBusy = true;
    }

    public void turnTo(double radians) {
        holdPoint(new Pose(getPose().getX(), getPose().getY(), radians));
        isTurning = true;
        isBusy = true;
    }

    public void turnToDegrees(double degrees) {
        turnTo(Math.toRadians(degrees));
    }

    public void turnDegrees(double degrees, boolean isLeft) {
        turn(Math.toRadians(degrees), isLeft);
    }

    public boolean isTurning() {
        return isTurning;
    }

    public boolean isBusy() {
        return isBusy;
    }

    public Pose getPose() {
        return tracker.getPose();
    }

    public void setPose(Pose pose) {
        tracker.setPose(pose);
    }

    public Vector getVelocity() {
        return tracker.getVelocity();
    }

    public Vector getAcceleration() {
        return tracker.getAcceleration();
    }

    public double getVelocityMagnitude() {
        return tracker.getVelocity().getMagnitude();
    }

    public void setStartingPose(Pose pose) {
        tracker.setStartingPose(pose);
    }

    public void setCurrentPoseWithOffset(Pose pose) {
        tracker.setCurrentPoseWithOffset(pose);
    }

    public void setXOffset(double xOffset) {
        tracker.setXOffset(xOffset);
    }

    public void setYOffset(double yOffset) {
        tracker.setYOffset(yOffset);
    }

    public void setHeadingOffset(double headingOffset) {
        tracker.setHeadingOffset(headingOffset);
    }

    public double getXOffset() {
        return tracker.getXOffset();
    }

    public double getYOffset() {
        return tracker.getYOffset();
    }

    public double getHeadingOffset() {
        return tracker.getHeadingOffset();
    }

    public void resetOffset() {
        tracker.resetOffset();
    }

    public double getTotalHeading() {
        return tracker.getTotalHeading();
    }

    public Path getCurrentPath() {
        return currentPath;
    }

    public DashboardPoseTracker getDashboardPoseTracker() {
        return dashboardPoseTracker;
    }

    public void drawOnDashBoard() {
        if (drawOnDashboard) {
            Drawing.drawDebug(this);
        }
    }

    public boolean isLocalizationNAN() {
        return tracker.getLocalizer().isNAN();
    }

    public double getHeadingError() {
        return errorHandler.getHeadingError();
    }

    public double getDriveError() {
        return errorHandler.getDriveError();
    }

    /**
     * This returns the t value of the closest point on the current Path to the robot
     * In the absence of a current Path, it returns 1.0.
     *
     * @return returns the current t value.
     */
    public double getCurrentTValue() {
        if (isBusy) return currentPath.getClosestPointTValue();
        return 1.0;
    }

    /**
     * This returns the current path number. For following Paths, this will return 0. For PathChains,
     * this will return the current path number. For holding Points, this will also return 0.
     *
     * @return returns the current path number.
     */
    public double getCurrentPathNumber() {
        if (!followingPathChain) return 0;
        return chainIndex;
    }

    /**
     * This returns a new PathBuilder object for easily building PathChains.
     *
     * @return returns a new PathBuilder object.
     */
    public PathBuilder pathBuilder() {
        return new PathBuilder();
    }


    /**
     * This writes out information about the various motion Vectors to the Telemetry specified.
     *
     * @param telemetry this is an instance of Telemetry or the FTC Dashboard telemetry that this
     *                  method will use to output the debug data.
     */
    public void telemetryDebug(Telemetry telemetry) {
        telemetryDebug(new MultipleTelemetry(telemetry));
    }


    /**
     * This resets the IMU, if applicable.
     */
    private void resetIMU() throws InterruptedException {
        tracker.resetIMU();
    }

    /**
     * This will update the PIDF coefficients for primary Heading PIDF mid run
     * can be used between paths
     *
     * @param set PIDF coefficients you would like to set.
     */
    public void setHeadingPIDF(CustomPIDFCoefficients set){
        errorHandler.setHeadingPIDF(set);
    }

    /**
     * This will update the PIDF coefficients for primary Translational PIDF mid run
     * can be used between paths
     *
     * @param set PIDF coefficients you would like to set.
     */
    public void setTranslationalPIDF(CustomPIDFCoefficients set){
        errorHandler.setTranslationalPIDF(set);
    }

    /**
     * This will update the PIDF coefficients for primary Drive PIDF mid run
     * can be used between paths
     *
     * @param set PIDF coefficients you would like to set.
     */
    public void setDrivePIDF(CustomFilteredPIDFCoefficients set){
        errorHandler.setDrivePIDF(set);
    }

    /**
     * This will update the PIDF coefficients for secondary Heading PIDF mid run
     * can be used between paths
     *
     * @param set PIDF coefficients you would like to set.
     */
    public void setSecondaryHeadingPIDF(CustomPIDFCoefficients set){
        errorHandler.setSecondaryHeadingPIDF(set);
    }

    /**
     * This will update the PIDF coefficients for secondary Translational PIDF mid run
     * can be used between paths
     *
     * @param set PIDF coefficients you would like to set.
     */
    public void setSecondaryTranslationalPIDF(CustomPIDFCoefficients set){
        errorHandler.setSecondaryTranslationalPIDF(set);
    }

    /**
     * This will update the PIDF coefficients for secondary Drive PIDF mid run
     * can be used between paths
     *
     * @param set PIDF coefficients you would like to set.
     */
    public void setSecondaryDrivePIDF(CustomFilteredPIDFCoefficients set){
        errorHandler.setSecondaryDrivePIDF(set);
    }

    /**
     * Checks if the robot is at a certain point within certain tolerances
     * @param point Point to compare with the current point
     * @param xTolerance Tolerance for the x position
     * @param yTolerance Tolerance for the y position
     */
    public boolean atPoint(Point point, double xTolerance, double yTolerance) {
        return Math.abs(point.getX() - getPose().getX()) < xTolerance && Math.abs(point.getY() - getPose().getY()) < yTolerance;
    }

    /**
     * Checks if the robot is at a certain pose within certain tolerances
     * @param pose Pose to compare with the current pose
     * @param xTolerance Tolerance for the x position
     * @param yTolerance Tolerance for the y position
     * @param headingTolerance Tolerance for the heading
     */
    public boolean atPose(Pose pose, double xTolerance, double yTolerance, double headingTolerance) {
        return Math.abs(pose.getX() - getPose().getX()) < xTolerance && Math.abs(pose.getY() - getPose().getY()) < yTolerance && Math.abs(pose.getHeading() - getPose().getHeading()) < headingTolerance;
    }

    /**
     * Checks if the robot is at a certain pose within certain tolerances
     * @param pose Pose to compare with the current pose
     * @param xTolerance Tolerance for the x position
     * @param yTolerance Tolerance for the y position
     */
    public boolean atPose(Pose pose, double xTolerance, double yTolerance) {
        return Math.abs(pose.getX() - getPose().getX()) < xTolerance && Math.abs(pose.getY() - getPose().getY()) < yTolerance;
    }

    /**
     * This gets a Point from the current Path from a specified t-value.
     *
     * @return returns the Point.
     */
    public Point getPointFromPath(double t) {
        if (currentPath != null) {
            return currentPath.getPoint(t);
        } else {
            return null;
        }
    }
}
