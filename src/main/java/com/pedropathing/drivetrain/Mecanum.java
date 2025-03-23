package com.pedropathing.drivetrain;

import static com.pedropathing.drivetrain.FollowerConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.pedropathing.util.MathFunctions;
import com.pedropathing.util.Vector;

import java.util.Arrays;
import java.util.List;

/**
 * This is the Mecanum class. This class handles the running of a mecanum drivetrain.
 * Mecanum drivetrains are a type of drivetrain that use mecanum wheels, which are wheels with rollers
 * at a 45 degree angle to the wheel's axis. This allows the robot to move in any direction without
 * changing the orientation of the robot. This class extends the Drivetrain class.
 *
 * @see Drivetrain
 * @version 1.0, 3/23/2025
 * @author Baron Henderson - 20077 The Indubitables
 */
public class Mecanum extends Drivetrain {
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;
    private double[] drivePowers;
    private double maxPowerScaling = 1;

    // Mecanum wheel vectors (normalized)
    private Vector[] mecanumVectors;

    public Mecanum(HardwareMap hardwareMap) {
        super(hardwareMap);
        drivePowers = new double[4];
    }

    @Override
    public void initialize() {
        // Initialize mecanum vectors based on the front left wheel's preferred drive vector
        Vector copiedFrontLeftVector = MathFunctions.normalizeVector(FollowerConstants.frontLeftVector);

        mecanumVectors = new Vector[]{
            new Vector(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getTheta()),
            new Vector(copiedFrontLeftVector.getMagnitude(), 2*Math.PI-copiedFrontLeftVector.getTheta()),
            new Vector(copiedFrontLeftVector.getMagnitude(), 2*Math.PI-copiedFrontLeftVector.getTheta()),
            new Vector(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getTheta())
        };

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
        
        leftFront.setDirection(leftFrontMotorDirection);
        leftRear.setDirection(leftRearMotorDirection);
        rightFront.setDirection(rightFrontMotorDirection);
        rightRear.setDirection(rightRearMotorDirection);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setMotorsToFloat();
    }

    @Override
    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
        this.maxPowerScaling = maxPower;
    }

    @Override
    public double getMaxPower() {
        return maxPower;
    }

    @Override
    public void setDrivePowers(double[] drivePowers) {
        this.drivePowers = applyVoltageCompensation(drivePowers);
        for (int i = 0; i < motors.size(); i++) {
            if (Math.abs(motors.get(i).getPower() - this.drivePowers[i]) > motorCachingThreshold) {
                motors.get(i).setPower(this.drivePowers[i]);
            }
        }
    }

    @Override
    public double[] getDrivePowers() {
        return drivePowers;
    }

    @Override
    public void setMotorsToBrake() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    @Override
    public void setMotorsToFloat() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    @Override
    public void stop() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }

    /**
     * Convert drive vectors to wheel powers using mecanum kinematics
     */
    public double[] getDrivePowers(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        // Clamp input vector magnitudes
        if (correctivePower.getMagnitude() > maxPowerScaling) correctivePower.setMagnitude(maxPowerScaling);
        if (headingPower.getMagnitude() > maxPowerScaling) headingPower.setMagnitude(maxPowerScaling);
        if (pathingPower.getMagnitude() > maxPowerScaling) pathingPower.setMagnitude(maxPowerScaling);

        // Calculate pathing vectors for each side
        Vector[] truePathingVectors = calculatePathingVectors(correctivePower, headingPower, pathingPower);

        // Scale pathing vectors
        truePathingVectors[0] = MathFunctions.scalarMultiplyVector(truePathingVectors[0], 2.0);
        truePathingVectors[1] = MathFunctions.scalarMultiplyVector(truePathingVectors[1], 2.0);

        // Calculate wheel powers using mecanum kinematics
        double[] wheelPowers = calculateWheelPowers(truePathingVectors, robotHeading);

        // Normalize wheel powers if needed
        normalizeWheelPowers(wheelPowers);

        return wheelPowers;
    }

    private Vector[] calculatePathingVectors(Vector correctivePower, Vector headingPower, Vector pathingPower) {
        Vector[] truePathingVectors = new Vector[2];

        if (correctivePower.getMagnitude() == maxPowerScaling) {
            truePathingVectors[0] = MathFunctions.copyVector(correctivePower);
            truePathingVectors[1] = MathFunctions.copyVector(correctivePower);
        } else {
            Vector leftSideVector = MathFunctions.subtractVectors(correctivePower, headingPower);
            Vector rightSideVector = MathFunctions.addVectors(correctivePower, headingPower);

            if (leftSideVector.getMagnitude() > maxPowerScaling || rightSideVector.getMagnitude() > maxPowerScaling) {
                double headingScalingFactor = Math.min(
                    findNormalizingScaling(correctivePower, headingPower),
                    findNormalizingScaling(correctivePower, MathFunctions.scalarMultiplyVector(headingPower, -1))
                );
                truePathingVectors[0] = MathFunctions.subtractVectors(
                    correctivePower,
                    MathFunctions.scalarMultiplyVector(headingPower, headingScalingFactor)
                );
                truePathingVectors[1] = MathFunctions.addVectors(
                    correctivePower,
                    MathFunctions.scalarMultiplyVector(headingPower, headingScalingFactor)
                );
            } else {
                Vector leftSideVectorWithPathing = MathFunctions.addVectors(leftSideVector, pathingPower);
                Vector rightSideVectorWithPathing = MathFunctions.addVectors(rightSideVector, pathingPower);

                if (leftSideVectorWithPathing.getMagnitude() > maxPowerScaling || 
                    rightSideVectorWithPathing.getMagnitude() > maxPowerScaling) {
                    double pathingScalingFactor = Math.min(
                        findNormalizingScaling(leftSideVector, pathingPower),
                        findNormalizingScaling(rightSideVector, pathingPower)
                    );
                    truePathingVectors[0] = MathFunctions.addVectors(
                        leftSideVector,
                        MathFunctions.scalarMultiplyVector(pathingPower, pathingScalingFactor)
                    );
                    truePathingVectors[1] = MathFunctions.addVectors(
                        rightSideVector,
                        MathFunctions.scalarMultiplyVector(pathingPower, pathingScalingFactor)
                    );
                } else {
                    truePathingVectors[0] = MathFunctions.copyVector(leftSideVectorWithPathing);
                    truePathingVectors[1] = MathFunctions.copyVector(rightSideVectorWithPathing);
                }
            }
        }

        return truePathingVectors;
    }

    private double[] calculateWheelPowers(Vector[] truePathingVectors, double robotHeading) {
        Vector[] mecanumVectorsCopy = new Vector[4];
        for (int i = 0; i < mecanumVectorsCopy.length; i++) {
            mecanumVectorsCopy[i] = MathFunctions.copyVector(mecanumVectors[i]);
            mecanumVectorsCopy[i].rotateVector(robotHeading);
        }

        double[] wheelPowers = new double[4];
        wheelPowers[0] = (mecanumVectorsCopy[1].getXComponent()*truePathingVectors[0].getYComponent() - 
                         truePathingVectors[0].getXComponent()*mecanumVectorsCopy[1].getYComponent()) / 
                        (mecanumVectorsCopy[1].getXComponent()*mecanumVectorsCopy[0].getYComponent() - 
                         mecanumVectorsCopy[0].getXComponent()*mecanumVectorsCopy[1].getYComponent());
        wheelPowers[1] = (mecanumVectorsCopy[0].getXComponent()*truePathingVectors[0].getYComponent() - 
                         truePathingVectors[0].getXComponent()*mecanumVectorsCopy[0].getYComponent()) / 
                        (mecanumVectorsCopy[0].getXComponent()*mecanumVectorsCopy[1].getYComponent() - 
                         mecanumVectorsCopy[1].getXComponent()*mecanumVectorsCopy[0].getYComponent());
        wheelPowers[2] = (mecanumVectorsCopy[3].getXComponent()*truePathingVectors[1].getYComponent() - 
                         truePathingVectors[1].getXComponent()*mecanumVectorsCopy[3].getYComponent()) / 
                        (mecanumVectorsCopy[3].getXComponent()*mecanumVectorsCopy[2].getYComponent() - 
                         mecanumVectorsCopy[2].getXComponent()*mecanumVectorsCopy[3].getYComponent());
        wheelPowers[3] = (mecanumVectorsCopy[2].getXComponent()*truePathingVectors[1].getYComponent() - 
                         truePathingVectors[1].getXComponent()*mecanumVectorsCopy[2].getYComponent()) / 
                        (mecanumVectorsCopy[2].getXComponent()*mecanumVectorsCopy[3].getYComponent() - 
                         mecanumVectorsCopy[3].getXComponent()*mecanumVectorsCopy[2].getYComponent());

        return wheelPowers;
    }

    private void normalizeWheelPowers(double[] wheelPowers) {
        double wheelPowerMax = Math.max(
            Math.max(Math.abs(wheelPowers[0]), Math.abs(wheelPowers[1])),
            Math.max(Math.abs(wheelPowers[2]), Math.abs(wheelPowers[3]))
        );
        if (wheelPowerMax > maxPowerScaling) {
            for (int i = 0; i < wheelPowers.length; i++) {
                wheelPowers[i] = (wheelPowers[i] / wheelPowerMax) * maxPowerScaling;
            }
        }
    }

    private double findNormalizingScaling(Vector staticVector, Vector variableVector) {
        double a = Math.pow(variableVector.getXComponent(), 2) + Math.pow(variableVector.getYComponent(), 2);
        double b = staticVector.getXComponent() * variableVector.getXComponent() + 
                   staticVector.getYComponent() * variableVector.getYComponent();
        double c = Math.pow(staticVector.getXComponent(), 2) + 
                   Math.pow(staticVector.getYComponent(), 2) - 
                   Math.pow(maxPowerScaling, 2);
        return (-b + Math.sqrt(Math.pow(b, 2) - a*c))/(a);
    }
}
