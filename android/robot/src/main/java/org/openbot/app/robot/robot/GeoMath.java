package org.openbot.app.robot.robot;

/** Small geographic helpers (WGS84 sphere approximation, fine for route-sized distances). */
public final class GeoMath {

    private static final double EARTH_RADIUS_M = 6371000.0;

    private GeoMath() {}

    /** Great-circle distance in metres. */
    public static double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
        double phi1 = Math.toRadians(lat1);
        double phi2 = Math.toRadians(lat2);
        double dPhi = phi2 - phi1;
        double dLambda = Math.toRadians(lon2 - lon1);
        double a =
                Math.sin(dPhi / 2) * Math.sin(dPhi / 2)
                        + Math.cos(phi1) * Math.cos(phi2) * Math.sin(dLambda / 2) * Math.sin(dLambda / 2);
        return 2 * EARTH_RADIUS_M * Math.asin(Math.min(1.0, Math.sqrt(a)));
    }

    /** Initial bearing from point 1 to point 2: degrees clockwise from true north, 0..360. */
    public static double bearingDegrees(double lat1, double lon1, double lat2, double lon2) {
        double phi1 = Math.toRadians(lat1);
        double phi2 = Math.toRadians(lat2);
        double dLambda = Math.toRadians(lon2 - lon1);
        double y = Math.sin(dLambda) * Math.cos(phi2);
        double x =
                Math.cos(phi1) * Math.sin(phi2) - Math.sin(phi1) * Math.cos(phi2) * Math.cos(dLambda);
        double bearing = Math.toDegrees(Math.atan2(y, x));
        return (bearing + 360.0) % 360.0;
    }

    /**
     * The point reached by travelling distanceM from (lat, lon) along bearingDeg (degrees clockwise
     * from true north).
     *
     * @return {latitude, longitude}
     */
    public static double[] destination(double lat, double lon, double bearingDeg, double distanceM) {
        double delta = distanceM / EARTH_RADIUS_M;
        double theta = Math.toRadians(bearingDeg);
        double phi1 = Math.toRadians(lat);
        double lambda1 = Math.toRadians(lon);
        double phi2 =
                Math.asin(
                        Math.sin(phi1) * Math.cos(delta) + Math.cos(phi1) * Math.sin(delta) * Math.cos(theta));
        double lambda2 =
                lambda1
                        + Math.atan2(
                        Math.sin(theta) * Math.sin(delta) * Math.cos(phi1),
                        Math.cos(delta) - Math.sin(phi1) * Math.sin(phi2));
        double longitude = (Math.toDegrees(lambda2) + 540.0) % 360.0 - 180.0;
        return new double[] {Math.toDegrees(phi2), longitude};
    }

    /** Smallest signed angle from a to b in degrees, in (-180, 180]. */
    public static double angleDifferenceDegrees(double a, double b) {
        double diff = (b - a) % 360.0;
        if (diff > 180.0) {
            diff -= 360.0;
        } else if (diff <= -180.0) {
            diff += 360.0;
        }
        return diff;
    }
}