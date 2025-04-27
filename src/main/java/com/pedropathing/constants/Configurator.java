package com.pedropathing.constants;

@FunctionalInterface
public interface Configurator<T> {
    void configure(T constants);
}