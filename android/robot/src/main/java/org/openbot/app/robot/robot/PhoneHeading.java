package org.openbot.app.robot.robot;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import timber.log.Timber;

/**
 * Robot heading measured by the phone's own rotation-vector sensor.
 *
 * <p>Two modes:
 *
 * <ul>
 *   <li>Relative (default): the game rotation vector (gyro + accelerometer, no magnetometer, so
 *       motors cannot disturb it). Its zero is arbitrary, so callers must only use DIFFERENCES.
 *   <li>Absolute: the standard rotation vector, which also uses the magnetometer, so its zero is
 *       magnetic north (X = east, Y = north). Motors and metal can disturb it.
 * </ul>
 *
 * <p>Heading is reported in degrees, counter-clockwise positive, measured from the world X axis
 * (east in absolute mode), in (-180, 180]. It is the direction the robot faces, derived from how
 * the phone is mounted (see {@link Mount}).
 *
 * <p>If the forward direction points almost straight up or down (phone mounted the wrong way), no
 * valid heading is produced, so {@link #hasFreshHeading(long)} stays false instead of returning a
 * meaningless value.
 *
 * <p>Sensor callbacks are delivered on the main thread.
 */
public class PhoneHeading implements SensorEventListener {

    /** How the phone is mounted on the robot. */
    public enum Mount {
        /** Phone upright, back camera pointing in the robot's forward direction (OpenBot default). */
        UPRIGHT_CAMERA_FORWARD,
        /** Phone lying flat, the top edge of the phone pointing in the forward direction. */
        FLAT_TOP_FORWARD
    }

    // The forward vector must have at least this horizontal length (0..1) to give a heading.
    private static final float MIN_HORIZONTAL = 0.25f;

    private final SensorManager sensorManager;
    private final Sensor sensor;
    private final Mount mount;
    private final Handler handler = new Handler(Looper.getMainLooper());
    private final float[] rotation = new float[9];

    private volatile float headingDeg = 0f;
    private volatile long lastValidAtMs = 0L;
    private boolean started = false;

    /** Relative heading (arbitrary zero). */
    public PhoneHeading(Context context, Mount mount) {
        this(context, mount, false);
    }

    /**
     * @param absolute true to use the magnetometer-based rotation vector (zero = magnetic north)
     */
    public PhoneHeading(Context context, Mount mount, boolean absolute) {
        this.mount = mount;
        this.sensorManager =
                (SensorManager) context.getApplicationContext().getSystemService(Context.SENSOR_SERVICE);
        Sensor found = null;
        if (sensorManager != null) {
            if (!absolute) {
                found = sensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);
            }
            if (found == null) {
                found = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
            }
        }
        this.sensor = found;
        if (found == null) {
            Timber.w("No rotation-vector sensor on this device: heading feedback unavailable");
        }
    }

    /** True if the device has a usable rotation-vector sensor. */
    public boolean isAvailable() {
        return sensor != null;
    }

    public void start() {
        if (started || sensor == null || sensorManager == null) {
            return;
        }
        started =
                sensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_GAME, handler);
        if (!started) {
            Timber.w("Could not register the heading sensor");
        }
    }

    public void stop() {
        if (!started || sensorManager == null) {
            return;
        }
        sensorManager.unregisterListener(this);
        started = false;
        lastValidAtMs = 0L;
    }

    /** True if a valid heading was received no longer than maxAgeMs ago. */
    public boolean hasFreshHeading(long maxAgeMs) {
        long last = lastValidAtMs;
        return last != 0L && SystemClock.elapsedRealtime() - last <= maxAgeMs;
    }

    /** Latest heading in degrees (counter-clockwise positive). Check {@link #hasFreshHeading}. */
    public float getHeadingDeg() {
        return headingDeg;
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        SensorManager.getRotationMatrixFromVector(rotation, event.values);

        // rotation transforms device coordinates to world coordinates (Z up).
        // Forward direction of the robot, expressed in the world frame:
        float forwardX;
        float forwardY;
        if (mount == Mount.UPRIGHT_CAMERA_FORWARD) {
            // Back camera direction = device -Z.
            forwardX = -rotation[2];
            forwardY = -rotation[5];
        } else {
            // Top edge of the phone = device +Y.
            forwardX = rotation[1];
            forwardY = rotation[4];
        }

        if (Math.hypot(forwardX, forwardY) < MIN_HORIZONTAL) {
            // Forward direction is (almost) vertical: the heading would be meaningless.
            return;
        }

        headingDeg = (float) Math.toDegrees(Math.atan2(forwardY, forwardX));
        lastValidAtMs = SystemClock.elapsedRealtime();
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {}
}