package org.openbot.app.robot.robot;

import android.os.SystemClock;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

/**
 * State of the route-teaching session (the operator drives manually while the robot records GPS
 * waypoints) and of the three route slots (Zone A, Zone B, Zone C).
 *
 * <p>Step 5: saved routes are stored on disk through {@link RouteStore}, so they survive app
 * restarts. The disk is the source of truth: routes are re-read with {@link #reloadFromStore()}.
 *
 * <p>An overwrite replaces the old route only when the NEW recording is finished, valid AND safely
 * written to disk. Cancelling a recording, losing the link, finishing a route that is too short,
 * or a failed disk write leaves the previously saved route untouched.
 */
public class RouteRecorder {

    public enum State {
        IDLE,
        RECORDING
    }

    public enum StartResult {
        STARTED,
        ALREADY_RECORDING,
        INVALID_ZONE,
        ZONE_EXISTS
    }

    public enum FinishResult {
        SAVED,
        NOT_RECORDING,
        TOO_SHORT,
        SAVE_FAILED
    }

    /** The three route slots. */
    public static final List<String> ZONES =
            Collections.unmodifiableList(Arrays.asList("Zone A", "Zone B", "Zone C"));

    public static final String ERROR_MISSION_ACTIVE = "MISSION_ACTIVE";
    public static final String ERROR_ALREADY_RECORDING = "ALREADY_RECORDING";
    public static final String ERROR_NOT_RECORDING = "NOT_RECORDING";
    public static final String ERROR_UNKNOWN_ACTION = "UNKNOWN_ACTION";
    public static final String ERROR_INVALID_ZONE = "INVALID_ZONE";
    public static final String ERROR_ZONE_EXISTS = "ZONE_EXISTS";
    public static final String ERROR_ROUTE_TOO_SHORT = "ROUTE_TOO_SHORT";
    public static final String ERROR_SAVE_FAILED = "SAVE_FAILED";
    public static final String ERROR_NO_ACTIVITY = "NO_ACTIVITY";

    /** A fix older than this no longer counts as "have a GPS fix". */
    private static final long FIX_STALE_MS = 5000L;

    private State state = State.IDLE;
    private String zone = "";
    private WaypointSampler sampler = null;
    private RouteStore store = null;

    // GPS status of the current recording.
    private String gpsSource = "";
    private float lastAccuracyM = -1f;
    private long lastFixAtMs = 0L;
    private boolean mockSeen = false;

    // Zones that have a saved route (sorted by name): a copy of what is on disk.
    private final Map<String, SavedRoute> routes = new TreeMap<>();

    public State getState() {
        return state;
    }

    public boolean isRecording() {
        return state == State.RECORDING;
    }

    /** Zone of the current recording, or "" when idle. */
    public String getZone() {
        return zone;
    }

    public boolean isZoneSaved(String zoneName) {
        return routes.containsKey(zoneName);
    }

    /** The saved route of a zone, or null. */
    public SavedRoute getRoute(String zoneName) {
        return routes.get(zoneName);
    }

    /** Sets where routes are stored. Without a store, no route can be saved. */
    public void setStore(RouteStore routeStore) {
        store = routeStore;
    }

    /** Re-reads the saved routes from disk (not while a recording is in progress). */
    public void reloadFromStore() {
        if (state == State.RECORDING || store == null) {
            return;
        }
        routes.clear();
        routes.putAll(store.loadAll(ZONES));
    }

    /**
     * Starts a recording session for a zone.
     *
     * @param overwrite true if the operator confirmed replacing an existing route for this zone
     */
    public StartResult start(String zoneName, boolean overwrite) {
        if (state == State.RECORDING) {
            return StartResult.ALREADY_RECORDING;
        }
        if (zoneName == null || !ZONES.contains(zoneName)) {
            return StartResult.INVALID_ZONE;
        }
        if (routes.containsKey(zoneName) && !overwrite) {
            return StartResult.ZONE_EXISTS;
        }
        state = State.RECORDING;
        zone = zoneName;
        sampler = new WaypointSampler();
        gpsSource = "";
        lastAccuracyM = -1f;
        lastFixAtMs = 0L;
        mockSeen = false;
        return StartResult.STARTED;
    }

    /** Records which kind of position source feeds this recording. */
    public void setGpsSource(String name) {
        gpsSource = name == null ? "" : name;
    }

    /**
     * Feeds one position fix into the recording.
     *
     * @param headingDeg current heading (any fixed zero) or NaN
     * @param moving true while the robot is being driven
     * @return true if the fix became a waypoint
     */
    public boolean onFix(
            double latitude,
            double longitude,
            float accuracyM,
            double headingDeg,
            boolean moving,
            boolean mock) {
        if (state != State.RECORDING || sampler == null) {
            return false;
        }
        lastAccuracyM = accuracyM;
        lastFixAtMs = SystemClock.elapsedRealtime();
        if (mock) {
            mockSeen = true;
        }
        return sampler.addFix(latitude, longitude, accuracyM, headingDeg, moving);
    }

    /**
     * Ends the session and saves the route to its zone, replacing any previous route there. The
     * route only counts as saved once it is safely on disk; a route that is too short, or a failed
     * write, leaves the previous route (if any) untouched.
     */
    public FinishResult finish() {
        if (state != State.RECORDING || sampler == null) {
            return FinishResult.NOT_RECORDING;
        }
        String finishedZone = zone;
        WaypointSampler.Result result = sampler.finish();
        String source = gpsSource;
        boolean mock = mockSeen;
        reset();

        if (!result.valid) {
            return FinishResult.TOO_SHORT;
        }

        SavedRoute route =
                new SavedRoute(
                        finishedZone,
                        result.waypoints,
                        source,
                        mock,
                        result.lengthM,
                        System.currentTimeMillis());

        if (store == null || !store.save(route)) {
            return FinishResult.SAVE_FAILED;
        }
        routes.put(finishedZone, route);
        return FinishResult.SAVED;
    }

    /**
     * Ends the session and discards the recording. A previously saved route for the zone is kept.
     *
     * @return false if no recording was in progress
     */
    public boolean cancel() {
        if (state != State.RECORDING) {
            return false;
        }
        reset();
        return true;
    }

    private void reset() {
        state = State.IDLE;
        zone = "";
        sampler = null;
        gpsSource = "";
        lastAccuracyM = -1f;
        lastFixAtMs = 0L;
        mockSeen = false;
    }

    /** NO_FIX, WEAK (fix, but too inaccurate to use) or OK. */
    private String gpsState() {
        boolean haveFix =
                lastFixAtMs != 0L && SystemClock.elapsedRealtime() - lastFixAtMs <= FIX_STALE_MS;
        if (!haveFix) {
            return "NO_FIX";
        }
        return lastAccuracyM <= WaypointSampler.MAX_ACCURACY_M ? "OK" : "WEAK";
    }

    /**
     * Status sent to Phone B.
     *
     * @param error error code answering the last command, or "" if it was accepted
     * @param savedZone zone a route was just saved to (only in the answer to a finish), or ""
     */
    public JSONObject toStatusJson(String error, String savedZone) throws JSONException {
        JSONObject status = new JSONObject();
        status.put("state", state.name());
        status.put("zone", zone);
        status.put("waypoints", sampler == null ? 0 : sampler.getWaypointCount());

        JSONArray saved = new JSONArray();
        for (String savedName : routes.keySet()) {
            saved.put(savedName);
        }
        status.put("saved_zones", saved);

        String justSaved = savedZone == null ? "" : savedZone;
        status.put("saved_zone", justSaved);
        SavedRoute savedRoute = justSaved.isEmpty() ? null : routes.get(justSaved);
        status.put("saved_waypoints", savedRoute == null ? 0 : savedRoute.waypoints.size());

        boolean recording = state == State.RECORDING;
        status.put("gps_source", recording ? gpsSource : "");
        status.put("gps_state", recording ? gpsState() : "NO_FIX");
        double accuracy = lastAccuracyM < 0f ? -1.0 : Math.min(999.0, lastAccuracyM);
        status.put("gps_accuracy_m", Math.round(accuracy * 10.0) / 10.0);
        status.put("gps_mock", recording && mockSeen);

        status.put("error", error == null ? "" : error);
        return status;
    }
}