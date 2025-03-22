package com.pedropathing.follower;

import com.pedropathing.pathgen.Vector;

/**
 * This class holds the vectors used for following a path.
 *
 * @author Rishi Ponnapalli - 19411 Tech Tigers
 */
public class FollowerVectors {
    public final Vector correctiveVector;
    public final Vector headingVector;
    public final Vector pathingVector;
    public final double robotHeading;

    /**
     * This creates a new FollowerVectors object.
     *
     * @param correctiveVector the corrective vector
     * @param headingVector the heading vector
     * @param pathingVector the pathing vector
     * @param robotHeading the robot's heading
     */
    public FollowerVectors(Vector correctiveVector, Vector headingVector,
                           Vector pathingVector, double robotHeading) {
        this.correctiveVector = correctiveVector;
        this.headingVector = headingVector;
        this.pathingVector = pathingVector;
        this.robotHeading = robotHeading;
    }
}
