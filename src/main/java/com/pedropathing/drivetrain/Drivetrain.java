package com.pedropathing.drivetrain;

import com.pedropathing.util.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    protected VoltageSensor voltageSensor;
    protected double voltage = 0;
    protected final ElapsedTime voltageTimer = new ElapsedTime();
    protected static final double NOMINAL_VOLTAGE = 12.0;
    protected static final double CACHE_INVALIDATE_SECONDS = 0.5;
    protected boolean isTeleop = false;

    public Drivetrain(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
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

    /**
     * Get the current battery voltage
     * @return current battery voltage
     */
    public double getVoltage() {
        if (voltageTimer.seconds() > CACHE_INVALIDATE_SECONDS) {
            refreshVoltage();
        }
        return voltage;
    }

    /**
     * Get the normalized voltage (current voltage / nominal voltage)
     * @return normalized voltage
     */
    public double getVoltageNormalized() {
        return getVoltage() / NOMINAL_VOLTAGE;
    }

    /**
     * Refresh the cached voltage value
     */
    protected void refreshVoltage() {
        voltage = voltageSensor.getVoltage();
        voltageTimer.reset();
    }

    /**
     * Set whether the drivetrain is in teleop mode
     * @param isTeleop true if in teleop mode, false if in autonomous mode
     */
    public void setTeleopMode(boolean isTeleop) {
        this.isTeleop = isTeleop;
    }

    /**
     * Apply voltage compensation to motor powers
     * @param powers array of motor powers to compensate
     * @return compensated motor powers
     */
    protected double[] applyVoltageCompensation(double[] powers) {
        if (powers == null) return null;
        
        // Only apply voltage compensation if enabled for the current mode
        if ((isTeleop && !com.pedropathing.drivetrain.FollowerConstants.useVoltageCompensationInTeleOp) ||
            (!isTeleop && !com.pedropathing.drivetrain.FollowerConstants.useVoltageCompensationInAuto)) {
            return powers;
        }
        
        double voltageScale = getVoltageNormalized();
        double[] compensatedPowers = new double[powers.length];
        
        for (int i = 0; i < powers.length; i++) {
            compensatedPowers[i] = powers[i] * voltageScale;
        }
        
        return compensatedPowers;
    }
}
