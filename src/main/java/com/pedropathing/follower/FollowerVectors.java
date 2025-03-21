package com.pedropathing.follower;

import com.pedropathing.pathgen.Vector;

public class FollowerVectors {
    public final Vector correctiveVector;
    public final Vector headingVector;
    public final Vector pathingVector;
    public final double robotHeading;

    public FollowerVectors(Vector correctiveVector, Vector headingVector,
                           Vector pathingVector, double robotHeading) {
        this.correctiveVector = correctiveVector;
        this.headingVector = headingVector;
        this.pathingVector = pathingVector;
        this.robotHeading = robotHeading;
    }
}
