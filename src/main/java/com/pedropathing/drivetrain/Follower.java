package com.pedropathing.drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Follower {
    private HardwareMap hardwareMap;

    public Follower(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
}
