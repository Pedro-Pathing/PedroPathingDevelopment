package com.pedropathing.constants;

public final class PathEndConstraints {
    public double velocityConstraint = 0.1;
    public double translationalConstraint = 0.1;
    public double headingConstraint = 0.07;
    public double tValueConstraint = 0.995;
    public double timeoutConstraint = 500;

    public PathEndConstraints(
            double velocityConstraint,
            double translationalConstraint,
            double headingConstraint,
            double tValueConstraint,
            double timeoutConstraint) {
        this.velocityConstraint = velocityConstraint;
        this.translationalConstraint = translationalConstraint;
        this.headingConstraint = headingConstraint;
        this.tValueConstraint = tValueConstraint;
        this.timeoutConstraint = timeoutConstraint;
    }
}
