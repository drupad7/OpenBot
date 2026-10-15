package org.openbot.app.robot.robot;

import android.app.Activity;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import androidx.annotation.Nullable;
import java.io.File;
import org.json.JSONException;
import org.json.JSONObject;
import org.openbot.app.robot.env.BotToControllerEventBus;
import org.openbot.app.robot.env.ControllerToBotEventBus;
import org.openbot.app.robot.utils.ConnectionUtils;
import timber.log.Timber;

/**
 * Handles the routeCmd messages from Phone B (query / start / finish / cancel route recording),
 * feeds GPS fixes into the recorder, and reports ROUTE_STATUS back.
 *
 * <p>It subscribes to the controller event bus under its own names: ControlsFragment ignores
 * routeCmd, so the mission command path is untouched. It also watches the normal driveCmd stream
 * (read only) to know whether the robot is being driven. Everything runs on the main thread except
 * the bus callbacks, which only write volatile fields. Recording never commands the motors; the
 * operator drives with the normal driveCmd.
 *
 * <p>Saved routes live on disk (RouteStore) and are re-read every time the screen resumes.
 */
public class RouteTeachingController {

    /** What this controller needs to know about its host fragment. */
    public interface Host {
        @Nullable
        Activity getHostActivity();

        /** True between the fragment's onResume and onPause. */
        boolean isActive();

        /** True while a mission is preparing, running, holding for an obstacle, or paused. */
        boolean isMissionBusy();
    }

    /**
     * DESK-TEST SWITCH. When true, positions come from a simulation that follows the slider values
     * (recording) or the mission's wheel commands (missions) instead of from the GPS. Keep it FALSE
     * for real use: simulated routes are labelled SIMULATED on both phones and cannot be driven with
     * real GPS (or the reverse), but they are not real coordinates.
     */
    public static final boolean USE_SIMULATED_GPS = true;

    // Must match FreeRoamFragment.PHONE_MOUNT (how the phone sits on the robot).
    private static final PhoneHeading.Mount PHONE_MOUNT =
            PhoneHeading.Mount.UPRIGHT_CAMERA_FORWARD;
    private static final long HEADING_STALE_MS = 500L;

    // Folder (inside the app's private files) holding one JSON file per saved zone.
    private static final String ROUTES_FOLDER = "routes";

    private static final String SUBSCRIBER = "FreeRoamRouteCmd";
    private static final String DRIVE_SUBSCRIBER = "FreeRoamRouteDrive";
    private static final String STATUS_KEY_ROUTE = "ROUTE_STATUS";

    // While recording, ROUTE_STATUS is also sent this often (GPS state, waypoint count).
    private static final long STATUS_PERIOD_MS = 1000L;
    // The robot counts as "being driven" for this long after the last non-zero drive command
    // (GPS fixes lag behind the real position).
    private static final long MOVING_GRACE_MS = 2000L;
    // Drive values smaller than this count as "not driving".
    private static final float MOVE_THRESHOLD = 0.05f;

    private final Host host;
    private final RouteRecorder recorder = new RouteRecorder();
    private final Handler handler = new Handler(Looper.getMainLooper());

    private RouteLocationSource locationSource = null;
    private PhoneHeading phoneHeading = null;

    // Written by the bus thread, read on the main thread.
    private volatile float driveLeft = 0f;
    private volatile float driveRight = 0f;
    private volatile long lastMoveAtMs = 0L;

    private final Runnable statusRunnable =
            new Runnable() {
                @Override
                public void run() {
                    if (!recorder.isRecording()) {
                        return;
                    }
                    emitStatus("", "");
                    handler.postDelayed(this, STATUS_PERIOD_MS);
                }
            };

    public RouteTeachingController(Host host) {
        this.host = host;
    }

    public boolean isRecording() {
        return recorder.isRecording();
    }

    /** The saved route of a zone (as loaded from disk), or null if the zone has none. */
    @Nullable
    public SavedRoute getSavedRoute(String zone) {
        return recorder.getRoute(zone);
    }

    /**
     * Starts listening for routeCmd messages (and for drive commands, read only), and loads the
     * saved routes from disk.
     */
    public void attach() {
        Activity activity = host.getHostActivity();
        if (activity != null) {
            recorder.setStore(new RouteStore(new File(activity.getFilesDir(), ROUTES_FOLDER)));
            recorder.reloadFromStore();
        }

        ControllerToBotEventBus.unsubscribe(SUBSCRIBER);
        ControllerToBotEventBus.subscribe(
                SUBSCRIBER,
                event -> {
                    Activity hostActivity = host.getHostActivity();
                    if (hostActivity == null) {
                        return;
                    }
                    hostActivity.runOnUiThread(() -> handle(event));
                },
                error -> Timber.e(error, "routeCmd subscription error"),
                event -> event.has("routeCmd"));

        ControllerToBotEventBus.unsubscribe(DRIVE_SUBSCRIBER);
        ControllerToBotEventBus.subscribe(
                DRIVE_SUBSCRIBER,
                this::onDriveEvent,
                error -> Timber.e(error, "driveCmd watcher error"),
                event -> event.has("driveCmd"));
    }

    /** Stops listening and stops GPS/heading. */
    public void detach() {
        ControllerToBotEventBus.unsubscribe(SUBSCRIBER);
        ControllerToBotEventBus.unsubscribe(DRIVE_SUBSCRIBER);
        stopSources();
    }

    /**
     * Discards an in-progress recording (controller disconnected, screen going away). A partial
     * route is never saved and a previously saved route is kept. Does not touch the motors.
     */
    public void abort(String reason) {
        stopSources();
        if (recorder.cancel()) {
            Timber.w("Route recording discarded: %s", reason);
            emitStatus("", "");
        }
    }

    private void onDriveEvent(JSONObject event) {
        JSONObject command = event.optJSONObject("driveCmd");
        if (command == null) {
            return;
        }
        float left = (float) command.optDouble("l", 0.0);
        float right = (float) command.optDouble("r", 0.0);
        driveLeft = left;
        driveRight = right;
        if (Math.abs(left) > MOVE_THRESHOLD || Math.abs(right) > MOVE_THRESHOLD) {
            lastMoveAtMs = SystemClock.elapsedRealtime();
        }
    }

    private void handle(JSONObject event) {
        if (!host.isActive()) {
            Timber.w("Ignoring routeCmd: fragment not active");
            return;
        }
        JSONObject command = event.optJSONObject("routeCmd");
        if (command == null) {
            return;
        }

        String action = command.optString("action", "");
        switch (action) {
            case "QUERY":
                emitStatus("", "");
                break;
            case "RECORD_START":
                startRecording(command.optString("zone", ""), command.optBoolean("overwrite", false));
                break;
            case "RECORD_STOP":
                finishRecording();
                break;
            case "RECORD_CANCEL":
                cancelRecording();
                break;
            default:
                Timber.w("Unknown routeCmd action '%s'", action);
                emitStatus(RouteRecorder.ERROR_UNKNOWN_ACTION, "");
                break;
        }
    }

    private void startRecording(String zone, boolean overwrite) {
        // A route can only be taught while no mission owns the motors.
        if (host.isMissionBusy()) {
            Timber.w("Route recording refused: a mission is active");
            emitStatus(RouteRecorder.ERROR_MISSION_ACTIVE, "");
            return;
        }

        switch (recorder.start(zone, overwrite)) {
            case STARTED:
                break;
            case ALREADY_RECORDING:
                emitStatus(RouteRecorder.ERROR_ALREADY_RECORDING, "");
                return;
            case INVALID_ZONE:
                Timber.w("Route recording refused: invalid zone '%s'", zone);
                emitStatus(RouteRecorder.ERROR_INVALID_ZONE, "");
                return;
            case ZONE_EXISTS:
                Timber.w("Route recording refused: %s already exists", zone);
                emitStatus(RouteRecorder.ERROR_ZONE_EXISTS, "");
                return;
        }

        // Recording needs positions: start them, or refuse (keeping any saved route).
        String sourceError = startSources();
        if (sourceError != null) {
            recorder.cancel();
            Timber.w("Route recording refused: %s", sourceError);
            emitStatus(sourceError, "");
            return;
        }

        Timber.i("Route recording started (%s, overwrite=%s, source=%s)",
                zone, overwrite, locationSource.getName());
        handler.removeCallbacks(statusRunnable);
        handler.postDelayed(statusRunnable, STATUS_PERIOD_MS);
        emitStatus("", "");
    }

    /** Starts the position source (and phone heading). Returns null or an error code. */
    private String startSources() {
        Activity activity = host.getHostActivity();
        if (activity == null) {
            return RouteRecorder.ERROR_NO_ACTIVITY;
        }
        stopSources();

        if (USE_SIMULATED_GPS) {
            locationSource =
                    new SimulatedLocationSource(
                            new SimulatedLocationSource.DriveSupplier() {
                                @Override
                                public float getLeft() {
                                    return driveLeft;
                                }

                                @Override
                                public float getRight() {
                                    return driveRight;
                                }
                            });
        } else {
            locationSource = new GpsLocationSource(activity);
            // The phone's own rotation sensor gives a reliable "the robot turned" signal.
            phoneHeading = new PhoneHeading(activity, PHONE_MOUNT);
            phoneHeading.start();
        }

        String error = locationSource.start(this::onFix);
        if (error != null) {
            stopSources();
            return error;
        }
        recorder.setGpsSource(locationSource.getName());
        return null;
    }

    private void stopSources() {
        handler.removeCallbacks(statusRunnable);
        if (locationSource != null) {
            locationSource.stop();
            locationSource = null;
        }
        if (phoneHeading != null) {
            phoneHeading.stop();
            phoneHeading = null;
        }
    }

    /** Current heading for turn detection: the simulation's, else the phone's, else NaN. */
    private double currentHeading() {
        if (locationSource != null) {
            double simulated = locationSource.getHeadingDeg();
            if (!Double.isNaN(simulated)) {
                return simulated;
            }
        }
        if (phoneHeading != null && phoneHeading.hasFreshHeading(HEADING_STALE_MS)) {
            return phoneHeading.getHeadingDeg();
        }
        return Double.NaN;
    }

    private void onFix(double latitude, double longitude, float accuracyM, boolean mock) {
        if (!recorder.isRecording()) {
            return;
        }
        boolean moving = SystemClock.elapsedRealtime() - lastMoveAtMs <= MOVING_GRACE_MS;
        boolean added =
                recorder.onFix(latitude, longitude, accuracyM, currentHeading(), moving, mock);
        if (added) {
            Timber.i("Waypoint recorded (accuracy %.1f m)", accuracyM);
            emitStatus("", "");
        }
    }

    private void finishRecording() {
        String zone = recorder.getZone();
        RouteRecorder.FinishResult result = recorder.finish();
        stopSources();
        switch (result) {
            case SAVED:
                Timber.i("Route saved to %s", zone);
                emitStatus("", zone);
                break;
            case TOO_SHORT:
                Timber.w("Route discarded: too short or too few waypoints");
                emitStatus(RouteRecorder.ERROR_ROUTE_TOO_SHORT, "");
                break;
            case SAVE_FAILED:
                Timber.e("Route for %s could not be written to storage", zone);
                emitStatus(RouteRecorder.ERROR_SAVE_FAILED, "");
                break;
            case NOT_RECORDING:
                emitStatus(RouteRecorder.ERROR_NOT_RECORDING, "");
                break;
        }
    }

    private void cancelRecording() {
        stopSources();
        if (!recorder.cancel()) {
            emitStatus(RouteRecorder.ERROR_NOT_RECORDING, "");
            return;
        }
        Timber.i("Route recording cancelled");
        emitStatus("", "");
    }

    private void emitStatus(String error, String savedZone) {
        try {
            BotToControllerEventBus.emitEvent(
                    ConnectionUtils.createStatus(
                            STATUS_KEY_ROUTE, recorder.toStatusJson(error, savedZone)));
        } catch (JSONException e) {
            Timber.w(e, "Could not build route status");
        }
    }
}