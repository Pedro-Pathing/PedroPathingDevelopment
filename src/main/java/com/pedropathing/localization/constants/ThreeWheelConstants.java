package com.pedropathing.localization.constants;

import static com.pedropathing.constants.ConstantsUtils.requireNotNull;
import static com.pedropathing.constants.ConstantsUtils.requireNotZero;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.constants.Configurator;
import com.pedropathing.localization.Encoder;

/**
 * This is the ThreeWheelConstants class. It holds constants and parameters for the Three Wheel Localizer.
 * Configurations are validated and initialized similarly to DriveEncoderConstants.
 *
 * @author Baron Henderson
 * @version 2.0, 12/24/2024
 */

public final class ThreeWheelConstants {

    public ThreeWheelConstants(Configurator<ThreeWheelConstants> configurator) {
        configurator.configure(this);
        requireNotNull(leftEncoder_HardwareMapName, "leftEncoder_HardwareMapName");
        requireNotNull(rightEncoder_HardwareMapName, "rightEncoder_HardwareMapName");
        requireNotNull(strafeEncoder_HardwareMapName, "strafeEncoder_HardwareMapName");
    }

    public double forwardTicksToInches = .001989436789;
    public double strafeTicksToInches = .001989436789;
    public double turnTicksToInches = .001989436789;
    public double leftY = 1;
    public double rightY = -1;
    public double strafeX = -2.5;
    public String leftEncoder_HardwareMapName;
    public String rightEncoder_HardwareMapName;
    public String strafeEncoder_HardwareMapName;
    public double leftEncoderDirection = Encoder.REVERSE;
    public double rightEncoderDirection = Encoder.REVERSE;
    public double strafeEncoderDirection = Encoder.FORWARD;
}