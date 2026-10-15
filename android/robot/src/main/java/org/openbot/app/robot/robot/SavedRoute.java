package org.openbot.app.robot.robot;

import java.util.List;

/** A route saved to a zone: the taught GPS waypoints plus where they came from. */
public final class SavedRoute {
    public final String zone;
    public final List<RouteWaypoint> waypoints;
    /** "GPS" or "SIMULATED". */
    public final String gpsSource;
    /** True if Android flagged at least one fix of the recording as a mock location. */
    public final boolean mockLocation;
    /** Total path length in metres. */
    public final double lengthM;
    public final long savedAtEpochMs;

    public SavedRoute(
            String zone,
            List<RouteWaypoint> waypoints,
            String gpsSource,
            boolean mockLocation,
            double lengthM,
            long savedAtEpochMs) {
        this.zone = zone;
        this.waypoints = waypoints;
        this.gpsSource = gpsSource;
        this.mockLocation = mockLocation;
        this.lengthM = lengthM;
        this.savedAtEpochMs = savedAtEpochMs;
    }
}