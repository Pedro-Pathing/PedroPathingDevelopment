package com.pedropathing.drivetrain.util;

public class Constants {
    public static Class<?> fConstants;
    public static Class<?> lConstants;

    public static void setConstants(Class<?> followerConstants, Class<?> localizerConstants) {
        fConstants = followerConstants;
        lConstants = localizerConstants;
        setup();
    }

    private static void setup() {
        try {
            Class.forName(fConstants.getName());
            Class.forName(lConstants.getName());
        } catch (ClassNotFoundException e) {
            e.printStackTrace();
        }
    }
}
