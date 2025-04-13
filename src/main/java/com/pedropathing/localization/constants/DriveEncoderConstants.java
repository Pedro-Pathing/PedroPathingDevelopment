package com.pedropathing.localization.constants;

import static com.pedropathing.constants.ConstantsUtils.requireNotZero;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.constants.Configurator;

/**
 * This is the DriveEncoderConstants class. It holds many constants and parameters for the Drive
 * Encoder Localizer.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 12/24/2024
 */

public final class DriveEncoderConstants {

    public DriveEncoderConstants(Configurator<DriveEncoderConstants> configurator) {
        configurator.configure(this);
        requireNotZero(leftFrontEncoderDirection, "leftFrontEncoderDirection");
        requireNotZero(leftRearEncoderDirection, "leftRearEncoderDirection");
        requireNotZero(rightFrontEncoderDirection, "rightFrontEncoderDirection");
        requireNotZero(rightRearEncoderDirection, "rightRearEncoderDirection");
    }

    /**
     * The number of inches per ticks of the encoder for forward movement
     * Default Value: 1
     */
    public double forwardTicksToInches = 1;

    /**
     * The number of inches per ticks of the encoder for lateral movement (strafing)
     * Default Value: 1
     */
    public double strafeTicksToInches = 1;

    /**
     * The number of inches per ticks of the encoder for turning
     * Default Value: 1
     */
    public double turnTicksToInches = 1;

    public double robot_Width = 1;
    public double robot_Length = 1;

    /**
     * The direction of the left front encoder
     * Default Value: Encoder.REVERSE
     */
    public double leftFrontEncoderDirection;

    /**
     * The direction of the right front encoder
     * Default Value: Encoder.FORWARD
     */
    public double rightFrontEncoderDirection;

    /**
     * The direction of the left rear encoder
     * Default Value: Encoder.REVERSE
     */
    public double leftRearEncoderDirection;

    /**
     * The direction of the right rear encoder
     * Default Value: Encoder.FORWARD
     */
    public double rightRearEncoderDirection;
}
