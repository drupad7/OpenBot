package org.openbot.app.robot.robot;

import android.os.Handler;
import android.os.Looper;

/**
 * DESK-TEST ONLY. Simulates a differential-drive robot that follows the left/right drive values,
 * so route recording AND route following can be tested indoors without GPS. It is selected by
 * USE_SIMULATED_GPS in RouteTeachingController, which is off by default, and everything it
 * produces is labelled SIMULATED so it can never be mistaken for real GPS data.
 *
 * <p>Its heading is a true compass bearing (clockwise from north), so no heading sensors are
 * needed in simulation.
 */
public class SimulatedLocationSource implements RouteLocationSource {

    /** Current left/right drive values, -1..1. */
    public interface DriveSupplier {
        float getLeft();

        float getRight();
    }

    // Arbitrary start position of the simulation (the same for every recording and mission).
    private static final double START_LAT = 13.0827;
    private static final double START_LON = 80.2707;
    private static final float ACCURACY_M = 3f;
    // Wheel values (0.25, 0.25) give 0.2 m/s; (0.25, -0.25) rotate at 45 degrees per second.
    private static final double MAX_SPEED_MPS = 0.8;
    private static final double MAX_TURN_RATE_DEG_PER_S = 180.0;
    private static final long TICK_MS = 200L;
    private static final int TICKS_PER_FIX = 5; // one fix per second

    private final DriveSupplier drive;
    private final Handler handler = new Handler(Looper.getMainLooper());

    private Listener listener;
    private boolean running = false;
    private double eastM = 0.0;
    private double northM = 0.0;
    private double headingDeg = 0.0; // compass bearing, clockwise from north
    private int tickCount = 0;

    private final Runnable tick =
            new Runnable() {
                @Override
                public void run() {
                    if (!running) {
                        return;
                    }
                    step();
                    tickCount++;
                    if (tickCount % TICKS_PER_FIX == 0) {
                        emitFix();
                    }
                    handler.postDelayed(this, TICK_MS);
                }
            };

    public SimulatedLocationSource(DriveSupplier drive) {
        this.drive = drive;
    }

    @Override
    public String start(Listener listener) {
        this.listener = listener;
        eastM = 0.0;
        northM = 0.0;
        headingDeg = 0.0;
        tickCount = 0;
        running = true;
        emitFix(); // an immediate fix, so the start point is captured at once
        handler.postDelayed(tick, TICK_MS);
        return null;
    }

    private void step() {
        double dt = TICK_MS / 1000.0;
        double left = drive.getLeft();
        double right = drive.getRight();

        double forward = (left + right) / 2.0 * MAX_SPEED_MPS;
        // Left wheel faster than the right wheel turns the robot to the right (clockwise).
        headingDeg = (headingDeg + (left - right) / 2.0 * MAX_TURN_RATE_DEG_PER_S * dt + 360.0) % 360.0;

        double rad = Math.toRadians(headingDeg);
        eastM += forward * Math.sin(rad) * dt;
        northM += forward * Math.cos(rad) * dt;
    }

    private void emitFix() {
        if (listener == null) {
            return;
        }
        double lat = START_LAT + northM / 111320.0;
        double lon = START_LON + eastM / (111320.0 * Math.cos(Math.toRadians(START_LAT)));
        listener.onFix(lat, lon, ACCURACY_M, false);
    }

    @Override
    public void stop() {
        running = false;
        handler.removeCallbacks(tick);
        listener = null;
    }

    @Override
    public String getName() {
        return "SIMULATED";
    }

    @Override
    public double getHeadingDeg() {
        return headingDeg;
    }
}