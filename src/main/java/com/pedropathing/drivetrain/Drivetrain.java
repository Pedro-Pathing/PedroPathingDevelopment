package com.pedropathing.drivetrain;

import com.pedropathing.util.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This is the Drivetrain class. This class handles the running of the drivetrain.
 * This class is abstract and is meant to be extended by specific drivetrain implementations.
 *
 * @version 1.0, 3/23/2025
 * @author Baron Henderson - 20077 The Indubitables
 */
public abstract class Drivetrain {
    protected HardwareMap hardwareMap;
    protected double maxPower = 1.0;

    public Drivetrain(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    /**
     * Initialize the drivetrain hardware
     */
    public abstract void initialize();

    /**
     * Set the maximum power for the drivetrain
     * @param maxPower power between 0 and 1
     */
    public abstract void setMaxPower(double maxPower);

    /**
     * Get the maximum power for the drivetrain
     * @return max power between 0 and 1
     */
    public abstract double getMaxPower();

    /**
     * Set the drive powers for the motors
     * @param drivePowers array of motor powers
     */
    public abstract void setDrivePowers(double[] drivePowers);

    /**
     * Get the drive powers for the motors
     * @return array of motor powers
     */
    public abstract double[] getDrivePowers();

    /**
     * Convert drive vectors to wheel powers using drivetrain-specific kinematics
     * @param correctivePower Vector for position/curvature correction
     * @param headingPower Vector for heading correction
     * @param pathingPower Vector for forward motion
     * @param robotHeading Current robot heading
     * @return Array of wheel powers
     */
    public abstract double[] getDrivePowers(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading);

    /**
     * Set the motors to brake mode
     */
    public abstract void setMotorsToBrake();

    /**
     * Set the motors to float mode
     */
    public abstract void setMotorsToFloat();

    /**
     * Stop all motors
     */
    public abstract void stop();
}
