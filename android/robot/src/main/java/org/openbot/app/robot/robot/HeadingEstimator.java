package org.openbot.app.robot.robot;

import java.util.ArrayList;
import java.util.List;

/**
 * Estimates the robot's heading as a compass bearing (degrees clockwise from true north, 0..360).
 *
 * <p>The phone's gyroscope-based rotation gives a smooth heading but with an unknown zero. This
 * class finds the offset between that arbitrary zero and true north:
 *
 * <ol>
 *   <li>From the magnetometer (compass) samples, if they agree with each other.
 *   <li>Otherwise from an assumption supplied by the caller (for example "the robot faces the
 *       direction the route starts"), see {@link #initFromAssumedBearing}.
 *   <li>Afterwards the offset is corrected whenever the robot has driven a straight stretch, by
 *       comparing the GPS course with the gyro heading (this also removes a bad initial offset and
 *       slow gyro drift).
 * </ol>
 *
 * <p>Plain Java on purpose (no Android classes), so it can be tested on a desktop.
 */
public class HeadingEstimator {

    public enum Source {
        NONE,
        MAGNETOMETER,
        ROUTE_ALIGNMENT,
        GPS_COURSE
    }

    /** Number of recent compass samples that are compared with each other. */
    public static final int COMPASS_WINDOW = 30;
    /** Compass samples needed before the compass is trusted. */
    public static final int COMPASS_MIN_SAMPLES = 15;
    /** The compass is only trusted if its samples spread less than this (degrees, circular std). */
    public static final double COMPASS_MAX_STD_DEG = 15.0;
    /** Straight distance needed for one GPS-course measurement. */
    public static final double BASELINE_M = 8.0;
    /** The gyro heading may change at most this much along a "straight" stretch. */
    public static final double STRAIGHT_TOLERANCE_DEG = 20.0;
    /** How much of each GPS-course measurement is blended into the offset (0..1). */
    public static final double GPS_BLEND = 0.3;
    /** A GPS-course measurement that disagrees with the current offset by more is ignored. */
    public static final double GPS_OUTLIER_DEG = 60.0;
    /** GPS fixes with a larger accuracy radius are not used for the course. */
    public static final double GPS_MAX_ACCURACY_M = 12.0;

    private double gyroBearing = Double.NaN; // gyro heading as a clockwise bearing, arbitrary zero
    private double offsetDeg = Double.NaN; // bearing = gyroBearing + offset
    private Source source = Source.NONE;
    private int gpsCorrections = 0;

    private final List<Double> compassOffsets = new ArrayList<>();

    private boolean anchorSet = false;
    private double anchorLat;
    private double anchorLon;
    private double anchorGyroBearing;

    public static double normalize360(double degrees) {
        double result = degrees % 360.0;
        return result < 0 ? result + 360.0 : result;
    }

    /** Circular mean of angles in degrees, 0..360. */
    static double circularMean(List<Double> angles) {
        double sumSin = 0;
        double sumCos = 0;
        for (double angle : angles) {
            sumSin += Math.sin(Math.toRadians(angle));
            sumCos += Math.cos(Math.toRadians(angle));
        }
        return normalize360(Math.toDegrees(Math.atan2(sumSin, sumCos)));
    }

    /** Circular standard deviation in degrees (small = the angles agree). */
    static double circularStdDeg(List<Double> angles) {
        double sumSin = 0;
        double sumCos = 0;
        for (double angle : angles) {
            sumSin += Math.sin(Math.toRadians(angle));
            sumCos += Math.cos(Math.toRadians(angle));
        }
        double r = Math.hypot(sumSin, sumCos) / angles.size();
        r = Math.min(1.0, Math.max(1e-9, r));
        return Math.toDegrees(Math.sqrt(-2.0 * Math.log(r)));
    }

    public void reset() {
        gyroBearing = Double.NaN;
        offsetDeg = Double.NaN;
        source = Source.NONE;
        gpsCorrections = 0;
        compassOffsets.clear();
        anchorSet = false;
    }

    /**
     * Feeds the gyro heading.
     *
     * @param relativeCcwDeg heading from the phone's rotation sensor: counter-clockwise positive,
     *     arbitrary zero
     */
    public void addGyro(double relativeCcwDeg) {
        gyroBearing = normalize360(-relativeCcwDeg);
    }

    /**
     * Feeds one compass sample.
     *
     * @param trueBearingDeg heading from the magnetometer, already corrected to true north
     */
    public void addCompass(double trueBearingDeg) {
        if (Double.isNaN(gyroBearing)) {
            return;
        }
        compassOffsets.add(normalize360(trueBearingDeg - gyroBearing));
        if (compassOffsets.size() > COMPASS_WINDOW) {
            compassOffsets.remove(0);
        }
        if (source == Source.NONE
                && compassOffsets.size() >= COMPASS_MIN_SAMPLES
                && circularStdDeg(compassOffsets) <= COMPASS_MAX_STD_DEG) {
            offsetDeg = circularMean(compassOffsets);
            source = Source.MAGNETOMETER;
        }
    }

    /** True if the compass samples agree with each other (so the compass could be trusted). */
    public boolean isCompassConsistent() {
        return compassOffsets.size() >= COMPASS_MIN_SAMPLES
                && circularStdDeg(compassOffsets) <= COMPASS_MAX_STD_DEG;
    }

    /**
     * Sets the offset from an assumed heading, only if no other reference exists yet. Use this when
     * the compass is not usable and the robot is known to face a certain direction.
     */
    public void initFromAssumedBearing(double assumedBearingDeg) {
        if (source != Source.NONE || Double.isNaN(gyroBearing)) {
            return;
        }
        offsetDeg = normalize360(assumedBearingDeg - gyroBearing);
        source = Source.ROUTE_ALIGNMENT;
    }

    /**
     * Feeds one GPS fix. Straight stretches give a GPS course that corrects the offset.
     *
     * @param moving true while the robot is driving forward
     */
    public void addGpsFix(double lat, double lon, float accuracyM, boolean moving) {
        if (Double.isNaN(gyroBearing) || !(accuracyM <= GPS_MAX_ACCURACY_M) || !moving) {
            anchorSet = false;
            return;
        }

        if (!anchorSet) {
            setAnchor(lat, lon);
            return;
        }

        // Not straight any more: start measuring again from here.
        if (Math.abs(GeoMath.angleDifferenceDegrees(anchorGyroBearing, gyroBearing))
                > STRAIGHT_TOLERANCE_DEG) {
            setAnchor(lat, lon);
            return;
        }

        double distance = GeoMath.distanceMeters(anchorLat, anchorLon, lat, lon);
        if (distance < BASELINE_M) {
            return;
        }

        double course = GeoMath.bearingDegrees(anchorLat, anchorLon, lat, lon);
        List<Double> gyros = new ArrayList<>();
        gyros.add(anchorGyroBearing);
        gyros.add(gyroBearing);
        double sample = normalize360(course - circularMean(gyros));

        if (source == Source.NONE) {
            offsetDeg = sample;
            source = Source.GPS_COURSE;
            gpsCorrections++;
        } else {
            double delta = GeoMath.angleDifferenceDegrees(offsetDeg, sample);
            if (Math.abs(delta) <= GPS_OUTLIER_DEG) {
                offsetDeg = normalize360(offsetDeg + GPS_BLEND * delta);
                gpsCorrections++;
            }
        }
        setAnchor(lat, lon);
    }

    private void setAnchor(double lat, double lon) {
        anchorSet = true;
        anchorLat = lat;
        anchorLon = lon;
        anchorGyroBearing = gyroBearing;
    }

    public boolean isReady() {
        return !Double.isNaN(offsetDeg) && !Double.isNaN(gyroBearing);
    }

    /** Current heading, degrees clockwise from true north, or NaN if not ready. */
    public double getBearingDeg() {
        return isReady() ? normalize360(gyroBearing + offsetDeg) : Double.NaN;
    }

    public Source getSource() {
        return source;
    }

    /** How many GPS-course corrections were applied so far. */
    public int getGpsCorrections() {
        return gpsCorrections;
    }
}