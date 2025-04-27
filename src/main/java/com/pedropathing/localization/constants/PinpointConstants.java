package com.pedropathing.localization.constants;

import static com.pedropathing.constants.ConstantsUtils.requireNotNull;
import static com.pedropathing.constants.ConstantsUtils.requireNotZero;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.constants.Configurator;
import com.pedropathing.localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This is the PinpointConstants class. It holds constants and parameters for the Pinpoint Localizer.
 * Configurations are validated and initialized similarly to DriveEncoderConstants.
 *
 * @author Baron Henderson
 * @version 2.0, 12/24/2024
 */
public final class PinpointConstants {

    public PinpointConstants(Configurator<PinpointConstants> configurator) {
        configurator.configure(this);
        requireNotNull(hardwareMapName, "hardwareMapName");
        requireNotNull(forwardEncoderDirection, "forwardEncoderDirection");
        requireNotNull(strafeEncoderDirection, "strafeEncoderDirection");
    }

    public double forwardY = 1;
    public double strafeX = -2.5;
    public DistanceUnit distanceUnit = DistanceUnit.INCH;
    public String hardwareMapName;
    public boolean useYawScalar = false;
    public double yawScalar = 1.0;
    public boolean useCustomEncoderResolution = false;
    public GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public double customEncoderResolution = 13.26291192;
    public GoBildaPinpointDriver.EncoderDirection forwardEncoderDirection;
    public GoBildaPinpointDriver.EncoderDirection strafeEncoderDirection;
}