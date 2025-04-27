package com.pedropathing.localization.constants;

import static com.pedropathing.constants.ConstantsUtils.requireNotNull;
import static com.pedropathing.constants.ConstantsUtils.requireNotZero;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.constants.Configurator;
import com.pedropathing.localization.Encoder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * This is the TwoWheelConstants class. It holds constants and parameters for the Two Wheel Localizer.
 * Configurations are validated and initialized similarly to DriveEncoderConstants.
 *
 * @author Baron Henderson
 * @version 2.0, 12/24/2024
 */

public final class TwoWheelConstants {

    public TwoWheelConstants(Configurator<TwoWheelConstants> configurator) {
        configurator.configure(this);
        requireNotNull(forwardEncoder_HardwareMapName, "forwardEncoder_HardwareMapName");
        requireNotNull(strafeEncoder_HardwareMapName, "strafeEncoder_HardwareMapName");
        requireNotNull(IMU_Orientation, "IMU_Orientation");
    }

    public double forwardTicksToInches = .001989436789;
    public double strafeTicksToInches = .001989436789;
    public double forwardY = 1;
    public double strafeX = -2.5;
    public String IMU_HardwareMapName = "imu";
    public String forwardEncoder_HardwareMapName;
    public String strafeEncoder_HardwareMapName;
    public RevHubOrientationOnRobot IMU_Orientation;
    public double forwardEncoderDirection = Encoder.REVERSE;
    public double strafeEncoderDirection = Encoder.FORWARD;
}