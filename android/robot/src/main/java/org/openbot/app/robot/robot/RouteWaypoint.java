package org.openbot.app.robot.robot;

/**
 * One waypoint of a taught route.
 *
 * <p>heading is the direction of travel at the waypoint: degrees clockwise from true north
 * (0..360), computed from the bearing between neighbouring waypoints. Accuracy is the GPS
 * accuracy (metres) of the fix the waypoint came from.
 */
public final class RouteWaypoint {
    public final int sequence;
    public final double latitude;
    public final double longitude;
    public final double heading;
    public final float accuracyM;

    public RouteWaypoint(
            int sequence, double latitude, double longitude, double heading, float accuracyM) {
        this.sequence = sequence;
        this.latitude = latitude;
        this.longitude = longitude;
        this.heading = heading;
        this.accuracyM = accuracyM;
    }
}