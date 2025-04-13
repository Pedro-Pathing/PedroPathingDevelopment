package com.pedropathing.constants;

import java.util.Objects;

public final class ConstantsUtils {

    private ConstantsUtils() {
    }

    public static <T> void requireNotNull(T object, String name) {
        Objects.requireNonNull(object, name + " must be initialized.");
    }

    public static void requireNotZero(double number, String name) {
        if (number == 0) throw new IllegalArgumentException(name + " must be initialized.");
    }
}
