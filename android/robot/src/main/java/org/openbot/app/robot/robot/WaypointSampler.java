package org.openbot.app.robot.robot;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Decides which GPS fixes become route waypoints while the operator drives the robot.
 *
 * <p>Goals: a useful path, not hundreds of near-identical samples, and no phantom waypoints from
 * GPS jitter. Rules:
 *
 * <ul>
 *   <li>Fixes with poor accuracy are ignored.
 *   <li>The first good fix is the start point.
 *   <li>Later waypoints are only added while the robot is being driven (a parked phone's GPS
 *       wanders by metres), when it is far enough from the previous waypoint (spacing grows with
 *       the GPS accuracy) or when its heading changed significantly after moving a little.
 *   <li>At the end, the latest good fix received while driving becomes the end point if it is
 *       clearly away from the last waypoint.
 * </ul>
 *
 * <p>Plain Java on purpose (no Android classes), so it can be tested on a desktop.
 */
public class WaypointSampler {

    /** Fixes with a worse (larger) accuracy radius than this are ignored. */
    public static final double MAX_ACCURACY_M = 15.0;
    /** Minimum distance between waypoints on a straight path. */
    public static final double MIN_SPACING_M = 5.0;
    /** The spacing is at least this many times the current GPS accuracy. */
    public static final double SPACING_ACCURACY_FACTOR = 2.0;
    /** A heading change of at least this many degrees (since the last waypoint) is a turn. */
    public static final double TURN_TRIGGER_DEG = 30.0;
    /** A turn only adds a waypoint after the robot moved at least this far. */
    public static final double MIN_MOVE_FOR_TURN_M = 3.0;
    /** The final fix is added as the end point only if it is at least this far from the last. */
    public static final double MIN_END_SEPARATION_M = 3.0;
    /** A route needs at least a start and an end point. */
    public static final int MIN_WAYPOINTS = 2;
    /** A route shorter than this (total path length) is rejected. */
    public static final double MIN_ROUTE_LENGTH_M = 5.0;

    private static final class Point {
        final double latitude;
        final double longitude;
        final float accuracyM;

        Point(double latitude, double longitude, float accuracyM) {
            this.latitude = latitude;
            this.longitude = longitude;
            this.accuracyM = accuracyM;
        }

        double distanceTo(Point other) {
            return GeoMath.distanceMeters(latitude, longitude, other.latitude, other.longitude);
        }
    }

    /** Outcome of finish(). */
    public static final class Result {
        public final boolean valid;
        public final String problem;
        public final List<RouteWaypoint> waypoints;
        public final double lengthM;

        Result(boolean valid, String problem, List<RouteWaypoint> waypoints, double lengthM) {
            this.valid = valid;
            this.problem = problem;
            this.waypoints = waypoints;
            this.lengthM = lengthM;
        }
    }

    private final List<Point> points = new ArrayList<>();
    // Latest good fix that arrived while the robot was being driven: the candidate end point.
    // (A fix from a parked robot is just GPS wander and must never become the end point.)
    private Point latestMovingFix = null;
    private double headingAtLastWaypoint = Double.NaN;

    /** Number of waypoints recorded so far (the end point is added by finish()). */
    public int getWaypointCount() {
        return points.size();
    }

    /**
     * Feeds one GPS fix.
     *
     * @param accuracyM GPS accuracy radius in metres
     * @param headingDeg current heading in degrees (any fixed zero; only CHANGES are used), or NaN
     * @param moving true while the robot is being driven
     * @return true if the fix became a waypoint
     */
    public boolean addFix(
            double latitude, double longitude, float accuracyM, double headingDeg, boolean moving) {
        if (!(accuracyM <= MAX_ACCURACY_M)) {
            return false; // also rejects NaN
        }

        Point fix = new Point(latitude, longitude, accuracyM);
        if (moving) {
            latestMovingFix = fix;
        }

        if (Double.isNaN(headingAtLastWaypoint) && !Double.isNaN(headingDeg)) {
            headingAtLastWaypoint = headingDeg;
        }

        if (points.isEmpty()) {
            points.add(fix);
            return true;
        }
        if (!moving) {
            return false;
        }

        Point last = points.get(points.size() - 1);
        double distance = last.distanceTo(fix);
        double spacing = Math.max(MIN_SPACING_M, SPACING_ACCURACY_FACTOR * accuracyM);
        boolean farEnough = distance >= spacing;

        boolean turned =
                !Double.isNaN(headingDeg)
                        && !Double.isNaN(headingAtLastWaypoint)
                        && Math.abs(GeoMath.angleDifferenceDegrees(headingAtLastWaypoint, headingDeg))
                        >= TURN_TRIGGER_DEG
                        && distance >= Math.max(MIN_MOVE_FOR_TURN_M, accuracyM);

        if (farEnough || turned) {
            points.add(fix);
            headingAtLastWaypoint = headingDeg;
            return true;
        }
        return false;
    }

    /** Ends the recording: adds the end point, computes headings and validates the route. */
    public Result finish() {
        List<Point> route = new ArrayList<>(points);

        if (!route.isEmpty() && latestMovingFix != null) {
            Point last = route.get(route.size() - 1);
            if (latestMovingFix != last && last.distanceTo(latestMovingFix) >= MIN_END_SEPARATION_M) {
                route.add(latestMovingFix);
            }
        }

        if (route.size() < MIN_WAYPOINTS) {
            return new Result(false, "TOO_FEW_WAYPOINTS", Collections.emptyList(), 0.0);
        }

        double length = 0.0;
        for (int i = 1; i < route.size(); i++) {
            length += route.get(i - 1).distanceTo(route.get(i));
        }
        if (length < MIN_ROUTE_LENGTH_M) {
            return new Result(false, "ROUTE_TOO_SHORT", Collections.emptyList(), length);
        }

        List<RouteWaypoint> result = new ArrayList<>();
        for (int i = 0; i < route.size(); i++) {
            // Heading = direction of travel: toward the next waypoint, and for the end point the
            // direction it was reached from.
            Point from = i < route.size() - 1 ? route.get(i) : route.get(i - 1);
            Point to = i < route.size() - 1 ? route.get(i + 1) : route.get(i);
            double heading =
                    GeoMath.bearingDegrees(from.latitude, from.longitude, to.latitude, to.longitude);
            Point point = route.get(i);
            result.add(
                    new RouteWaypoint(i, point.latitude, point.longitude, heading, point.accuracyM));
        }
        return new Result(true, "", Collections.unmodifiableList(result), length);
    }
}