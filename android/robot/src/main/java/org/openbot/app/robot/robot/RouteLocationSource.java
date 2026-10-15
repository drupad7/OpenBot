package org.openbot.app.robot.robot;

/** A provider of position fixes for route recording: real GPS, or a desk-test simulation. */
public interface RouteLocationSource {

    String ERROR_NO_PERMISSION = "NO_LOCATION_PERMISSION";
    String ERROR_GPS_DISABLED = "GPS_DISABLED";

    interface Listener {
        /**
         * @param accuracyM accuracy radius in metres
         * @param mock true if Android flags the fix as coming from a mock location provider
         */
        void onFix(double latitude, double longitude, float accuracyM, boolean mock);
    }

    /** Starts delivering fixes (on the main thread). Returns null on success or an error code. */
    String start(Listener listener);

    void stop();

    /** "GPS" or "SIMULATED". */
    String getName();

    /** Robot heading in degrees if this source knows it (the simulation does), otherwise NaN. */
    double getHeadingDeg();
}