package org.openbot.app.robot.robot;

import java.util.List;

/**
 * Follows a taught route of GPS waypoints: for the current position and heading it gives the
 * distance to the next waypoint, the bearing to it, and the heading error that the steering has to
 * remove. It also decides when a waypoint counts as reached.
 *
 * <p>Phone GPS is only accurate to metres, so a waypoint is reached when the robot is inside a
 * radius that grows with the reported accuracy (but never so large that it swallows the next
 * waypoint). Close to a waypoint the position is too noisy to steer by, so the last good bearing
 * is kept. A watchdog reports when the robot makes no progress toward the waypoint.
 *
 * <p>It only produces a heading error; turning that into wheel commands is the job of
 * {@link DifferentialSteering}. Plain Java on purpose (no Android classes), so it can be tested on
 * a desktop.
 */
public class GpsNavigator {

    /** Arrival radius = this many times the GPS accuracy ... */
    public static final double ARRIVE_ACCURACY_FACTOR = 1.2;
    /** ... but at least this (metres) ... */
    public static final double ARRIVE_MIN_M = 3.0;
    /** ... and at most this. */
    public static final double ARRIVE_MAX_M = 8.0;
    /** The radius never exceeds this fraction of the leg length (so no waypoint is skipped). */
    public static final double ARRIVE_LEG_FRACTION = 0.45;
    /** Absolute floor for the arrival radius. */
    public static final double ARRIVE_FLOOR_M = 1.5;
    /** Inside this distance (or the GPS accuracy, if larger) the last good bearing is kept. */
    public static final double BEARING_LOCK_MIN_M = 2.0;
    /** No progress for this long is a fault. */
    public static final long NO_PROGRESS_TIMEOUT_MS = 90_000L;
    /** The distance to the target must shrink by this much to count as progress. */
    public static final double PROGRESS_GAIN_M = 2.0;

    /** Result of one update. */
    public static final class Step {
        public final boolean complete;
        public final boolean arrivedAtWaypoint;
        public final int reachedCount;
        public final int targetIndex;
        public final double distanceM;
        public final double bearingDeg;
        public final double headingErrorDeg;
        public final boolean noProgress;

        Step(
                boolean complete,
                boolean arrivedAtWaypoint,
                int reachedCount,
                int targetIndex,
                double distanceM,
                double bearingDeg,
                double headingErrorDeg,
                boolean noProgress) {
            this.complete = complete;
            this.arrivedAtWaypoint = arrivedAtWaypoint;
            this.reachedCount = reachedCount;
            this.targetIndex = targetIndex;
            this.distanceM = distanceM;
            this.bearingDeg = bearingDeg;
            this.headingErrorDeg = headingErrorDeg;
            this.noProgress = noProgress;
        }
    }

    private final List<RouteWaypoint> route;
    private int target = 0;
    private double lastBearing = Double.NaN;
    private double bestDistance = Double.MAX_VALUE;
    private long bestAtMs = 0L;

    public GpsNavigator(List<RouteWaypoint> route, long nowMs) {
        this.route = route;
        this.bestAtMs = nowMs;
    }

    public int getReachedCount() {
        return target;
    }

    public int getTotal() {
        return route.size();
    }

    /** Restarts the no-progress watchdog (call while the robot is deliberately held still). */
    public void markProgressNow(long nowMs) {
        bestAtMs = nowMs;
        bestDistance = Double.MAX_VALUE;
    }

    public static double arrivalRadius(List<RouteWaypoint> route, int index, double accuracyM) {
        double radius =
                Math.max(ARRIVE_MIN_M, Math.min(ARRIVE_MAX_M, ARRIVE_ACCURACY_FACTOR * accuracyM));
        if (index > 0) {
            RouteWaypoint previous = route.get(index - 1);
            RouteWaypoint current = route.get(index);
            double leg =
                    GeoMath.distanceMeters(
                            previous.latitude, previous.longitude, current.latitude, current.longitude);
            radius = Math.min(radius, ARRIVE_LEG_FRACTION * leg);
        }
        return Math.max(ARRIVE_FLOOR_M, radius);
    }

    /**
     * @param headingBearingDeg current heading, degrees clockwise from true north
     * @return the situation; headingErrorDeg is positive when the target is to the right (clockwise)
     */
    public Step update(
            double lat, double lon, float accuracyM, double headingBearingDeg, long nowMs) {
        boolean arrived = false;

        if (target < route.size()) {
            RouteWaypoint waypoint = route.get(target);
            double distance =
                    GeoMath.distanceMeters(lat, lon, waypoint.latitude, waypoint.longitude);
            if (distance <= arrivalRadius(route, target, accuracyM)) {
                target++;
                arrived = true;
                lastBearing = Double.NaN;
                markProgressNow(nowMs);
            }
        }

        if (target >= route.size()) {
            return new Step(true, arrived, target, target, 0.0, Double.NaN, 0.0, false);
        }

        RouteWaypoint waypoint = route.get(target);
        double distance = GeoMath.distanceMeters(lat, lon, waypoint.latitude, waypoint.longitude);

        double bearing;
        double lockRadius = Math.max(BEARING_LOCK_MIN_M, accuracyM);
        if (distance < lockRadius && !Double.isNaN(lastBearing)) {
            bearing = lastBearing; // too close for the noisy position to steer by
        } else {
            bearing = GeoMath.bearingDegrees(lat, lon, waypoint.latitude, waypoint.longitude);
            lastBearing = bearing;
        }

        double error =
                Double.isNaN(headingBearingDeg)
                        ? 0.0
                        : GeoMath.angleDifferenceDegrees(headingBearingDeg, bearing);

        if (distance < bestDistance - PROGRESS_GAIN_M) {
            bestDistance = distance;
            bestAtMs = nowMs;
        }
        boolean noProgress = nowMs - bestAtMs > NO_PROGRESS_TIMEOUT_MS;

        return new Step(false, arrived, target, target, distance, bearing, error, noProgress);
    }
}