package com.pedropathing.localization.constants;

import static com.pedropathing.constants.ConstantsUtils.requireNotNull;
import static com.pedropathing.constants.ConstantsUtils.requireNotZero;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.constants.Configurator;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This is the OTOSConstants class. It holds many constants and parameters for the OTOS Localizer.
 * Configurations are validated and initialized similar to DriveEncoderConstants.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/24/2024
 */

public final class OTOSConstants {

    public OTOSConstants(Configurator<OTOSConstants> configurator) {
        configurator.configure(this);
        requireNotNull(hardwareMapName, "hardwareMapName");
    }

    /**
     * Whether to use the corrected OTOS class.
     * Default Value: false
     */
    public boolean useCorrectedOTOSClass = false;

    /**
     * The name of the OTOS sensor in the hardware map.
     * Default Value: "sensor_otos"
     */
    public String hardwareMapName;

    /**
     * The linear unit of the OTOS sensor.
     * Default Value: DistanceUnit.INCH
     */
    public DistanceUnit linearUnit = DistanceUnit.INCH;

    /**
     * The angle unit of the OTOS sensor.
     * Default Value: AngleUnit.RADIANS
     */
    public AngleUnit angleUnit = AngleUnit.RADIANS;

    /**
     * The offset of the OTOS sensor from the center of the robot (x, y, heading).
     * Default Value: new Pose2D(0, 0, Math.PI / 2)
     */
    public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, Math.PI / 2);

    /**
     * The linear scalar of the OTOS sensor.
     * Used to scale distance measurements from the sensor.
     * Default Value: 1.0
     */
    public double linearScalar = 1.0;

    /**
     * The angular scalar of the OTOS sensor.
     * Used to scale angular measurements from the sensor.
     * Default Value: 1.0
     */
    public double angularScalar = 1.0;
}