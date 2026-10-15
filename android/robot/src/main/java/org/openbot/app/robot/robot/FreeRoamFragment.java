package org.openbot.app.robot.robot;

import static org.openbot.app.robot.utils.Enums.ControlMode;
import static org.openbot.app.robot.utils.Enums.DriveMode;
import static org.openbot.app.robot.utils.Enums.SpeedMode;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.content.res.ColorStateList;
import android.hardware.GeomagneticField;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.navigation.Navigation;
import com.github.anastr.speedviewlib.components.Section;
import com.google.android.material.internal.ViewUtils;
import java.util.Locale;
import java.util.UUID;
import org.jetbrains.annotations.NotNull;
import org.json.JSONException;
import org.json.JSONObject;
import org.openbot.app.robot.R;
import org.openbot.app.robot.common.ControlsFragment;
import org.openbot.app.robot.databinding.FragmentFreeRoamBinding;
import org.openbot.app.robot.env.BotToControllerEventBus;
import org.openbot.app.robot.env.PhoneController;
import org.openbot.app.robot.utils.ConnectionUtils;
import org.openbot.app.robot.utils.Constants;
import org.openbot.app.robot.utils.Enums;
import org.openbot.app.robot.utils.PermissionUtils;
import timber.log.Timber;

public class FreeRoamFragment extends ControlsFragment {
    private FragmentFreeRoamBinding binding;
    private PhoneController phoneController;

    // ---------------------------------------------------------------------------
    // Route teaching. All routeCmd handling lives in RouteTeachingController; this fragment only
    // tells it whether it is active and whether a mission owns the motors.
    // ---------------------------------------------------------------------------

    private final RouteTeachingController routeTeaching =
            new RouteTeachingController(
                    new RouteTeachingController.Host() {
                        @Nullable
                        @Override
                        public Activity getHostActivity() {
                            return getActivity();
                        }

                        @Override
                        public boolean isActive() {
                            return fragmentActive;
                        }

                        @Override
                        public boolean isMissionBusy() {
                            return missionState == MissionState.PREPARING
                                    || missionState == MissionState.RUNNING
                                    || missionState == MissionState.OBSTACLE_DETECTED
                                    || missionState == MissionState.PAUSED;
                        }
                    });

    // ---------------------------------------------------------------------------
    // Autonomous mission: follows a saved GPS route.
    //
    //   GPS fix + heading + target waypoint
    //        -> GpsNavigator       (target bearing, heading error)
    //        -> DifferentialSteering (left / right wheel commands, 2WD)
    //
    // Heading comes from the phone's gyro, tied to true north by the compass (if consistent), by
    // the assumption "the robot faces the way the route starts", and then corrected from the GPS
    // course while driving straight (HeadingEstimator). Sonar obstacle stop, pause, stop,
    // disconnect and manual takeover work as before.
    // ---------------------------------------------------------------------------

    /** Mission states. Further states are added in later phases. */
    private enum MissionState {
        IDLE,
        PREPARING,
        RUNNING,
        OBSTACLE_DETECTED,
        PAUSED,
        MISSION_COMPLETE,
        ERROR
    }

    /** Outcome of one navigation step. */
    private enum NavResult {
        CONTINUE,
        ROUTE_COMPLETE,
        FAULT
    }

    /** Outcome of one PREPARING step. */
    private enum PrepResult {
        WAITING,
        READY,
        FAULT
    }

    private static final String ZONE_NOT_SELECTED = "Not Selected";

    /** Status key sent to Phone B inside the standard {"status":{...}} message. */
    private static final String STATUS_KEY_MISSION = "MISSION_STATUS";

    // Mission loop period. Matches the 100 ms sonar report interval set in Vehicle.
    private static final long MISSION_LOOP_MS = 100L;
    // How often a status message is sent while nothing else changed.
    private static final long STATUS_PERIOD_MS = 1000L;

    // ---- Heading sensors. INITIAL VALUES: tune on the real robot. ----
    // How the phone is mounted on the robot. Change this if the phone is mounted differently.
    private static final PhoneHeading.Mount PHONE_MOUNT =
            PhoneHeading.Mount.UPRIGHT_CAMERA_FORWARD;
    // If no valid heading sensor reading arrives for this long, stop (ERROR).
    private static final long HEADING_STALE_MS = 500L;

    // ---- GPS navigation. INITIAL VALUES: tune on the real robot. ----
    // A fix older than this no longer counts: the robot is held still until a new one arrives.
    private static final long GPS_FIX_STALE_MS = 3000L;
    // No good fix for this long is a fault (ERROR GPS_TIMEOUT).
    private static final long GPS_LOST_TIMEOUT_MS = 20000L;
    // Fixes with a larger accuracy radius than this (metres) are not used for navigation.
    private static final double MAX_NAV_ACCURACY_M = 20.0;
    // The mission only starts if the robot is within this distance of the route's first waypoint.
    private static final double MAX_START_DISTANCE_M = 50.0;
    // PREPARING must finish within this time (waiting for GPS and a heading reference).
    private static final long PREPARE_TIMEOUT_MS = 60000L;
    // How long to wait for a consistent compass before using the route-alignment fallback.
    private static final long COMPASS_WAIT_MS = 8000L;
    // Fallback when the compass is unusable: assume the robot faces the way the route starts.
    // Only allowed when the robot is this close to the route start (metres).
    private static final boolean ALLOW_ROUTE_ALIGNMENT_FALLBACK = true;
    private static final double ALIGNMENT_START_MAX_M = 15.0;

    // ---- Obstacle safety. INITIAL VALUES: they must be tuned on the real robot. ----
    // Sonar readings are in centimetres (see RobotInfoFragment).
    // Motion is withheld IMMEDIATELY when a valid reading is below the stop distance.
    // The sonar only looks forward: it does not protect the sides while turning in place.
    private static final float OBSTACLE_STOP_CM = 40f;
    // Reading must be at least this far to leave OBSTACLE_DETECTED (hysteresis).
    private static final float OBSTACLE_CLEAR_CM = 55f;
    // Reading must stay below the stop distance this long before the state changes to
    // OBSTACLE_DETECTED (motion is already stopped meanwhile; this only debounces the state).
    private static final long OBSTACLE_CONFIRM_MS = 200L;
    // Reading must stay clear this long before the mission continues automatically.
    private static final long OBSTACLE_CLEAR_MS = 500L;
    // If the firmware reports a sonar but no sonar message arrives for this long, stop (ERROR).
    private static final long SONAR_STALE_MS = 1000L;
    // If true, a mission cannot start without a sonar. Left false so the software can be
    // exercised without hardware; set to true for real autonomous runs.
    private static final boolean REQUIRE_SONAR_FOR_MISSION = false;

    // ---- Object detection records (MVP: ultrasonic, unclassified). ----
    // Minimum time between two recorded detections (duplicate prevention).
    private static final long DETECTION_COOLDOWN_MS = 5000L;

    private MissionState missionState = MissionState.IDLE;
    // Single authoritative mission zone value.
    private String missionZone = ZONE_NOT_SELECTED;
    private String missionError = "";
    // Identifies one mission run; kept across pause/resume, new on every fresh start.
    private String missionId = "";
    // Number of route waypoints reached so far (0..missionTotalWaypoints).
    private int missionWaypoint = 0;
    private int missionTotalWaypoints = 0;
    private int detectionCount = 0;
    private long missionStartTime = 0L;
    // Moment the timer was frozen (PAUSED, MISSION_COMPLETE, ERROR).
    private long missionPausedAt = 0L;

    // Route following.
    private SavedRoute missionRoute = null;
    private GpsNavigator navigator = null;
    private final DifferentialSteering steering = new DifferentialSteering();
    private final HeadingEstimator headingEstimator = new HeadingEstimator();
    private RouteLocationSource missionLocationSource = null;
    private PhoneHeading gyroHeading = null; // relative heading (gyro)
    private PhoneHeading compassHeading = null; // absolute heading (magnetometer)
    private double declinationDeg = 0.0;
    private boolean declinationKnown = false;

    // Latest good GPS fix.
    private boolean fixSeen = false;
    private double lastFixLat = 0.0;
    private double lastFixLon = 0.0;
    private float lastFixAccuracyM = -1f;
    private long lastGoodFixAtMs = 0L;

    // Last wheel command sent by the mission (also what the simulated robot follows).
    private float commandedLeft = 0f;
    private float commandedRight = 0f;

    private long prepareStartedAt = 0L;
    private boolean startDistanceChecked = false;
    private double startDistanceM = -1.0;
    private double navDistanceM = -1.0;
    private double navHeadingErrorDeg = Double.NaN;
    private String navFault = "";

    // Mission loop bookkeeping (SystemClock.elapsedRealtime based).
    private long lastStatusAt = 0L;
    private long sonarWatchStartedAt = 0L;
    private long lastSonarMessageAt = 0L;
    private long obstacleBelowSince = 0L;
    private long obstacleClearSince = 0L;
    private long lastDetectionAt = 0L;

    private MissionDetectionLog detectionLog;

    // True only between onResume() and onPause()/onDestroyView().
    // Mission commands are ignored and the mission loop will not drive while false.
    private boolean fragmentActive = false;

    // One handler, one runnable, always on the main thread.
    private final Handler missionHandler = new Handler(Looper.getMainLooper());

    @Override
    public View onCreateView(
            @NotNull LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        binding = FragmentFreeRoamBinding.inflate(inflater, container, false);
        return binding.getRoot();
    }

    @SuppressLint("RestrictedApi")
    @Override
    public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);

        phoneController = PhoneController.getInstance(requireContext());

        BotToControllerEventBus.emitEvent(
                ConnectionUtils.createStatus(
                        "FRAGMENT_TYPE", Enums.FragmentType.FREEROAM.getFragment()));
        if (vehicle == null) {
            mViewModel
                    .getVehicle()
                    .observe(
                            getViewLifecycleOwner(),
                            v -> {
                                if (v != null && vehicle == null) {
                                    vehicle = v;
                                    setupFreeRoamUi();
                                }
                            });
            return;
        }
        setupFreeRoamUi();
    }

    private void setupFreeRoamUi() {
        if (vehicle == null || binding == null) {
            return;
        }

        binding.voltageInfo.setText(getString(R.string.voltageInfo, "--.-"));
        binding.controllerContainer.speedInfo.setText(getString(R.string.speedInfo, "---,---"));
        binding.sonarInfo.setText(getString(R.string.distanceInfo, "---"));
        String connectionType = vehicle.getConnectionType();
        if (connectionType != null && connectionType.equals("USB")) {
            binding.usbToggle.setVisibility(View.VISIBLE);
            binding.bleToggle.setVisibility(View.GONE);
        } else if (connectionType != null && connectionType.equals("Bluetooth")) {
            binding.bleToggle.setVisibility(View.VISIBLE);
            binding.usbToggle.setVisibility(View.GONE);
        }

        setSpeedMode(SpeedMode.getByID(preferencesManager.getSpeedMode()));
        setControlMode(ControlMode.getByID(preferencesManager.getControlMode()));
        setDriveMode(DriveMode.getByID(preferencesManager.getDriveMode()));

        binding.controllerContainer.controlMode.setOnClickListener(
                v -> {
                    ControlMode controlMode = ControlMode.getByID(preferencesManager.getControlMode());
                    if (controlMode != null) setControlMode(Enums.switchControlMode(controlMode));
                });
        binding.controllerContainer.driveMode.setOnClickListener(
                v -> setDriveMode(Enums.switchDriveMode(vehicle.getDriveMode())));

        binding.controllerContainer.speedMode.setOnClickListener(
                v ->
                        setSpeedMode(
                                Enums.toggleSpeed(
                                        Enums.Direction.CYCLIC.getValue(),
                                        SpeedMode.getByID(preferencesManager.getSpeedMode()))));

        binding.speed.getSections().clear();
        binding.speed.addSections(
                new Section(
                        0f,
                        0.7f,
                        getResources().getColor(R.color.green),
                        ViewUtils.dpToPx(requireContext(), 24)),
                new Section(
                        0.7f,
                        0.8f,
                        getResources().getColor(R.color.yellow),
                        ViewUtils.dpToPx(requireContext(), 24)),
                new Section(
                        0.8f,
                        1.0f,
                        getResources().getColor(R.color.red),
                        ViewUtils.dpToPx(requireContext(), 24)));

        mViewModel
                .getUsbStatus()
                .observe(getViewLifecycleOwner(), status -> binding.usbToggle.setChecked(status));

        binding.usbToggle.setChecked(vehicle.isUsbConnected());
        binding.bleToggle.setChecked(vehicle.bleConnected());

        binding.usbToggle.setOnClickListener(
                v -> {
                    binding.usbToggle.setChecked(vehicle.isUsbConnected());
                    Navigation.findNavController(requireView()).navigate(R.id.open_usb_fragment);
                });
        binding.bleToggle.setOnClickListener(
                v -> {
                    binding.bleToggle.setChecked(vehicle.bleConnected());
                    Navigation.findNavController(requireView()).navigate(R.id.open_bluetooth_fragment);
                });
    }

    @Override
    public void onResume() {
        super.onResume();
        fragmentActive = true;
        routeTeaching.attach();
        if (vehicle != null && binding != null) {
            binding.bleToggle.setChecked(vehicle.bleConnected());
        }
    }

    @Override
    public void onPause() {
        // Mission must never keep driving in the background.
        fragmentActive = false;
        abortMission("fragment paused");
        routeTeaching.detach();
        routeTeaching.abort("fragment paused");
        super.onPause();
    }

    @Override
    public void onDestroyView() {
        fragmentActive = false;
        abortMission("view destroyed");
        routeTeaching.detach();
        routeTeaching.abort("view destroyed");
        super.onDestroyView();
    }

    @Override
    protected void processUSBData(String data) {
        // Record when the firmware last sent a sonar message ('s' header). This is used by the
        // mission loop to detect a silent/stale sonar. Must happen before any early return.
        if (data != null && !data.isEmpty() && data.charAt(0) == 's') {
            lastSonarMessageAt = SystemClock.elapsedRealtime();
        }

        if (vehicle == null || binding == null) {
            return;
        }

        binding.controllerContainer.speedInfo.setText(
                getString(
                        R.string.speedInfo,
                        String.format(
                                Locale.US, "%3.0f,%3.0f", vehicle.getLeftWheelRpm(), vehicle.getRightWheelRpm())));

        binding.voltageInfo.setText(
                getString(
                        R.string.voltageInfo, String.format(Locale.US, "%2.1f", vehicle.getBatteryVoltage())));
        binding.battery.setProgress(vehicle.getBatteryPercentage());
        if (vehicle.getBatteryPercentage() < 15) {
            binding.battery.setProgressTintList(
                    ColorStateList.valueOf(getResources().getColor(R.color.red)));
            binding.battery.setProgressBackgroundTintList(
                    ColorStateList.valueOf(getResources().getColor(R.color.red)));
        } else {
            binding.battery.setProgressTintList(
                    ColorStateList.valueOf(getResources().getColor(R.color.green)));
            binding.battery.setProgressBackgroundTintList(
                    ColorStateList.valueOf(getResources().getColor(R.color.green)));
        }

        binding.sonar.setProgress((int) (vehicle.getSonarReading() / 3));
        if (vehicle.getSonarReading() / 3 < 15) {
            binding.sonar.setProgressTintList(
                    ColorStateList.valueOf(getResources().getColor(R.color.red)));
        } else if (vehicle.getSonarReading() / 3 < 45) {
            binding.sonar.setProgressTintList(
                    ColorStateList.valueOf(getResources().getColor(R.color.yellow)));
        } else {
            binding.sonar.setProgressTintList(
                    ColorStateList.valueOf(getResources().getColor(R.color.green)));
        }

        binding.sonarInfo.setText(
                getString(
                        R.string.distanceInfo, String.format(Locale.US, "%3.0f", vehicle.getSonarReading())));
    }

    private void toggleIndicator(int value) {
        binding.indicatorRight.clearAnimation();
        binding.indicatorLeft.clearAnimation();
        binding.indicatorRight.setVisibility(View.INVISIBLE);
        binding.indicatorLeft.setVisibility(View.INVISIBLE);

        if (value == Enums.VehicleIndicator.RIGHT.getValue()) {
            binding.indicatorRight.startAnimation(startAnimation);
            binding.indicatorRight.setVisibility(View.VISIBLE);
        } else if (value == Enums.VehicleIndicator.LEFT.getValue()) {
            binding.indicatorLeft.startAnimation(startAnimation);
            binding.indicatorLeft.setVisibility(View.VISIBLE);
        }
    }

    protected void handleDriveCommand() {
        float left = vehicle.getLeftSpeed();
        float right = vehicle.getRightSpeed();
        binding.controllerContainer.controlInfo.setText(
                String.format(Locale.US, "%.0f,%.0f", left, right));

        binding.speed.speedPercentTo(vehicle.getSpeedPercent());

        binding.steering.setRotation(vehicle.getRotation());

        binding.driveGear.setText(vehicle.getDriveGear());
    }

    private void setSpeedMode(SpeedMode speedMode) {
        if (speedMode != null) {
            switch (speedMode) {
                case SLOW:
                    binding.controllerContainer.speedMode.setImageResource(R.drawable.ic_speed_low);
                    break;
                case NORMAL:
                    binding.controllerContainer.speedMode.setImageResource(R.drawable.ic_speed_medium);
                    break;
                case FAST:
                    binding.controllerContainer.speedMode.setImageResource(R.drawable.ic_speed_high);
                    break;
            }

            Timber.d("Updating  controlSpeed: %s", speedMode);
            preferencesManager.setSpeedMode(speedMode.getValue());
            vehicle.setSpeedMultiplier(speedMode.getValue());
        }
    }

    private void setControlMode(ControlMode controlMode) {
        if (controlMode != null) {
            switch (controlMode) {
                case GAMEPAD:
                    binding.controllerContainer.controlMode.setImageResource(R.drawable.ic_controller);
                    disconnectPhoneController();
                    break;
                case PHONE:
                    binding.controllerContainer.controlMode.setImageResource(R.drawable.ic_phone);
                    if (!PermissionUtils.hasControllerPermissions(requireActivity()))
                        requestPermissionLauncher.launch(Constants.PERMISSIONS_CONTROLLER);
                    else connectPhoneController();
                    break;
                case WEBSERVER:
                    binding.controllerContainer.controlMode.setImageResource(R.drawable.ic_server);
                    if (!PermissionUtils.hasControllerPermissions(requireActivity()))
                        requestPermissionLauncher.launch(Constants.PERMISSIONS_CONTROLLER);
                    else {
                        connectWebController();
                    }
                    break;
            }
            Timber.d("Updating  controlMode: %s", controlMode);
            preferencesManager.setControlMode(controlMode.getValue());
        }
    }

    protected void setDriveMode(DriveMode driveMode) {
        if (driveMode != null) {
            switch (driveMode) {
                case DUAL:
                    binding.controllerContainer.driveMode.setImageResource(R.drawable.ic_dual);
                    break;
                case GAME:
                    binding.controllerContainer.driveMode.setImageResource(R.drawable.ic_game);
                    break;
                case JOYSTICK:
                    binding.controllerContainer.driveMode.setImageResource(R.drawable.ic_joystick);
                    break;
            }

            Timber.d("Updating  driveMode: %s", driveMode);
            vehicle.setDriveMode(driveMode);
            preferencesManager.setDriveMode(driveMode.getValue());
        }
    }

    private void connectPhoneController() {
        phoneController.connect(requireContext());
        DriveMode oldDriveMode = currentDriveMode;
        // Currently only dual drive mode supported
        setDriveMode(DriveMode.DUAL);
        binding.controllerContainer.driveMode.setAlpha(0.5f);
        binding.controllerContainer.driveMode.setEnabled(false);
        preferencesManager.setDriveMode(oldDriveMode.getValue());
    }

    private void connectWebController() {
        phoneController.connectWebServer();
        Enums.DriveMode oldDriveMode = currentDriveMode;
        // Currently only dual drive mode supported
        setDriveMode(Enums.DriveMode.GAME);
        binding.controllerContainer.driveMode.setAlpha(0.5f);
        binding.controllerContainer.driveMode.setEnabled(false);
        preferencesManager.setDriveMode(oldDriveMode.getValue());
    }

    private void disconnectPhoneController() {
        phoneController.disconnect();
        setDriveMode(DriveMode.getByID(preferencesManager.getDriveMode()));
        binding.controllerContainer.driveMode.setEnabled(true);
        binding.controllerContainer.driveMode.setAlpha(1.0f);
    }

    @Override
    protected void processControllerKeyData(String commandType) {
        switch (commandType) {
            case Constants.CMD_DRIVE:
                // ControlsFragment has already applied the operator's drive values to the vehicle.
                // Any manual drive input takes over from the mission (mission goes to PAUSED and the
                // operator's command is left in place). During route teaching no mission is running,
                // so manual driving is never interrupted.
                if (isMissionMoving()) {
                    manualOverrideMission();
                }
                handleDriveCommand();
                break;

            case Constants.CMD_INDICATOR_LEFT:
            case Constants.CMD_INDICATOR_RIGHT:
            case Constants.CMD_INDICATOR_STOP:
                toggleIndicator(vehicle.getIndicator());
                break;

            case Constants.CMD_DRIVE_MODE:
                setDriveMode(Enums.switchDriveMode(vehicle.getDriveMode()));
                break;

            case Constants.CMD_DISCONNECTED:
                // Invalidate the mission loop BEFORE anything else so that no queued
                // mission tick can drive the motors again after the disconnect.
                abortMission("controller disconnected");
                routeTeaching.abort("controller disconnected");
                handleDriveCommand();
                setControlMode(ControlMode.GAMEPAD);
                break;

            case Constants.CMD_SPEED_DOWN:
                setSpeedMode(
                        Enums.toggleSpeed(
                                Enums.Direction.DOWN.getValue(),
                                Enums.SpeedMode.getByID(preferencesManager.getSpeedMode())));
                break;

            case Constants.CMD_SPEED_UP:
                setSpeedMode(
                        Enums.toggleSpeed(
                                Enums.Direction.UP.getValue(),
                                Enums.SpeedMode.getByID(preferencesManager.getSpeedMode())));
                break;

            case Constants.CMD_MISSION_START:
                startMission();
                break;

            case Constants.CMD_MISSION_PAUSE:
                pauseMission();
                break;

            case Constants.CMD_MISSION_STOP:
                stopMission();
                break;
        }
    }

    // ---------------------------------------------------------------------------
    // Mission control. All methods run on the main thread (controller events are
    // delivered via runOnUiThread, USB data, GPS and sensor events via the main thread, and the
    // handler uses the main looper), so there is no race between a command and a queued
    // mission loop.
    // ---------------------------------------------------------------------------

    /** True while the mission loop owns the motors (preparing, driving or holding for an obstacle). */
    private boolean isMissionMoving() {
        return missionState == MissionState.PREPARING
                || missionState == MissionState.RUNNING
                || missionState == MissionState.OBSTACLE_DETECTED;
    }

    /** Commands zero motor output. Safe to call at any time. */
    private void stopVehicle() {
        commandedLeft = 0f;
        commandedRight = 0f;
        if (vehicle != null) {
            vehicle.setControl(0f, 0f);
        }
    }

    /** Sends a left/right wheel command and remembers it. */
    private void driveVehicle(float left, float right) {
        commandedLeft = left;
        commandedRight = right;
        if (vehicle != null) {
            vehicle.setControl(left, right);
        }
    }

    /** Reads the zone from the last START command, or "" if none was supplied. */
    private String readRequestedZone() {
        if (lastControllerEvent == null) {
            return "";
        }
        JSONObject missionCmd = lastControllerEvent.optJSONObject("missionCmd");
        if (missionCmd == null) {
            return "";
        }
        return missionCmd.optString("zone", "");
    }

    private void updateMissionZone(String zone) {
        if (zone == null || zone.isEmpty()) {
            missionZone = ZONE_NOT_SELECTED;
        } else {
            missionZone = zone;
        }
    }

    private void resetMissionRuntime() {
        missionId = "";
        missionWaypoint = 0;
        missionTotalWaypoints = 0;
        detectionCount = 0;
        missionStartTime = 0L;
        missionPausedAt = 0L;
        missionError = "";
        obstacleBelowSince = 0L;
        obstacleClearSince = 0L;
        lastDetectionAt = 0L;

        missionRoute = null;
        navigator = null;
        steering.reset();
        headingEstimator.reset();
        declinationDeg = 0.0;
        declinationKnown = false;

        fixSeen = false;
        lastFixLat = 0.0;
        lastFixLon = 0.0;
        lastFixAccuracyM = -1f;
        lastGoodFixAtMs = 0L;

        commandedLeft = 0f;
        commandedRight = 0f;

        prepareStartedAt = 0L;
        startDistanceChecked = false;
        startDistanceM = -1.0;
        navDistanceM = -1.0;
        navHeadingErrorDeg = Double.NaN;
        navFault = "";

        updateMissionZone(null);
    }

    /** Mission elapsed time in seconds; frozen while PAUSED, MISSION_COMPLETE or ERROR. */
    private long missionElapsedSeconds() {
        if (missionStartTime == 0L) {
            return 0L;
        }
        long end = System.currentTimeMillis();
        boolean frozen =
                missionState == MissionState.PAUSED
                        || missionState == MissionState.MISSION_COMPLETE
                        || missionState == MissionState.ERROR;
        if (frozen && missionPausedAt != 0L) {
            end = missionPausedAt;
        }
        return Math.max(0L, (end - missionStartTime) / 1000L);
    }

    private static void putNumber(JSONObject json, String key, double value) throws JSONException {
        if (Double.isNaN(value) || Double.isInfinite(value)) {
            json.put(key, JSONObject.NULL);
        } else {
            json.put(key, Math.round(value * 100.0) / 100.0);
        }
    }

    private boolean isSimulatedGps() {
        return RouteTeachingController.USE_SIMULATED_GPS;
    }

    /** Current heading as a compass bearing (clockwise from true north), or NaN if unknown. */
    private double currentHeadingBearing() {
        if (isSimulatedGps()) {
            return missionLocationSource != null ? missionLocationSource.getHeadingDeg() : Double.NaN;
        }
        return headingEstimator.getBearingDeg();
    }

    private String headingSourceName() {
        if (isSimulatedGps()) {
            return "SIMULATED";
        }
        return headingEstimator.getSource().name();
    }

    /**
     * Sends the current mission state to Phone B as
     * {"status":{"MISSION_STATUS":"{\"state\":...}"}} using the existing status message path.
     * Failure to report must never affect motor safety.
     */
    private void emitMissionStatus() {
        try {
            int sonarCm = -1; // -1 = sonar unavailable or no valid reading
            if (vehicle != null && vehicle.isHasSonar()) {
                float reading = vehicle.getSonarReading();
                if (reading > 0f) {
                    sonarCm = Math.round(reading);
                }
            }

            JSONObject status = new JSONObject();
            status.put("state", missionState.name());
            status.put("zone", missionZone);
            // Waypoints REACHED so far (0..total).
            status.put("waypoint", missionWaypoint);
            status.put("total", missionTotalWaypoints);
            status.put("detections", detectionCount);
            status.put("elapsed_s", missionElapsedSeconds());
            status.put("sonar_cm", sonarCm);
            status.put("error", missionError);
            // Navigation details (Phone B does not need them yet, but they help debugging).
            putNumber(status, "distance_m", navDistanceM < 0 ? Double.NaN : navDistanceM);
            putNumber(status, "heading_error_deg", navHeadingErrorDeg);
            putNumber(status, "heading_deg", currentHeadingBearing());
            status.put("heading_source", headingSourceName());
            putNumber(status, "gps_accuracy_m", lastFixAccuracyM < 0 ? Double.NaN : lastFixAccuracyM);
            BotToControllerEventBus.emitEvent(ConnectionUtils.createStatus(STATUS_KEY_MISSION, status));
        } catch (JSONException e) {
            Timber.w(e, "Could not build mission status");
        }
    }

    /**
     * Records one MVP object detection (ultrasonic, unclassified) for the current mission. A
     * cooldown prevents the same object from being recorded repeatedly. Never affects motion.
     */
    private void recordDetection(float distanceCm, long nowRealtime) {
        if (lastDetectionAt != 0L && nowRealtime - lastDetectionAt < DETECTION_COOLDOWN_MS) {
            Timber.d("Detection skipped (cooldown)");
            return;
        }
        try {
            if (detectionLog == null) {
                detectionLog = new MissionDetectionLog(requireContext());
            }

            boolean haveFix =
                    lastGoodFixAtMs != 0L && nowRealtime - lastGoodFixAtMs <= GPS_LOST_TIMEOUT_MS;
            double heading = currentHeadingBearing();
            double robotLat = Double.NaN;
            double robotLon = Double.NaN;
            double objectLat = Double.NaN;
            double objectLon = Double.NaN;
            float accuracy = Float.NaN;
            String source = "UNAVAILABLE";

            if (haveFix) {
                robotLat = lastFixLat;
                robotLon = lastFixLon;
                accuracy = lastFixAccuracyM;
                source = isSimulatedGps() ? "SIMULATED" : "GPS";
                // The object is straight ahead of the robot, at the sonar distance.
                if (!Double.isNaN(heading)) {
                    double[] object = GeoMath.destination(robotLat, robotLon, heading, distanceCm / 100.0);
                    objectLat = object[0];
                    objectLon = object[1];
                } else {
                    objectLat = robotLat;
                    objectLon = robotLon;
                }
            }

            JSONObject record =
                    detectionLog.append(
                            missionId,
                            missionZone,
                            missionWaypoint,
                            distanceCm,
                            robotLat,
                            robotLon,
                            heading,
                            accuracy,
                            source,
                            objectLat,
                            objectLon);
            if (record != null) {
                lastDetectionAt = nowRealtime;
                detectionCount++;
                Timber.i("Detection recorded: %s", record.toString());
            }
        } catch (RuntimeException e) {
            // Recording must never interfere with robot safety.
            Timber.w(e, "Could not record detection");
        }
    }

    // ---------------------------------------------------------------------------
    // Mission sources: GPS (or the desk-test simulation) and the heading sensors. They run only
    // while a mission is active (including PAUSED, so that resuming is immediate).
    // ---------------------------------------------------------------------------

    /** Starts position and heading sources. Returns null on success or an error code. */
    private String startMissionSources() {
        Activity activity = getActivity();
        if (activity == null) {
            return "NO_ACTIVITY";
        }
        stopMissionSources();

        if (isSimulatedGps()) {
            // The simulated robot follows exactly the wheel commands the mission sends.
            missionLocationSource =
                    new SimulatedLocationSource(
                            new SimulatedLocationSource.DriveSupplier() {
                                @Override
                                public float getLeft() {
                                    return commandedLeft;
                                }

                                @Override
                                public float getRight() {
                                    return commandedRight;
                                }
                            });
        } else {
            missionLocationSource = new GpsLocationSource(activity);
            gyroHeading = new PhoneHeading(activity, PHONE_MOUNT);
            gyroHeading.start();
            compassHeading = new PhoneHeading(activity, PHONE_MOUNT, true);
            compassHeading.start();
        }

        String error = missionLocationSource.start(this::onMissionFix);
        if (error != null) {
            stopMissionSources();
            return error;
        }
        return null;
    }

    private void stopMissionSources() {
        if (missionLocationSource != null) {
            missionLocationSource.stop();
            missionLocationSource = null;
        }
        if (gyroHeading != null) {
            gyroHeading.stop();
            gyroHeading = null;
        }
        if (compassHeading != null) {
            compassHeading.stop();
            compassHeading = null;
        }
    }

    private void onMissionFix(double latitude, double longitude, float accuracyM, boolean mock) {
        if (!(accuracyM <= MAX_NAV_ACCURACY_M)) {
            return; // too inaccurate to navigate by; the last good fix keeps ageing
        }
        long now = SystemClock.elapsedRealtime();
        fixSeen = true;
        lastFixLat = latitude;
        lastFixLon = longitude;
        lastFixAccuracyM = accuracyM;
        lastGoodFixAtMs = now;

        if (!declinationKnown) {
            // Magnetic declination: the angle between magnetic north and true north at this place.
            GeomagneticField field =
                    new GeomagneticField(
                            (float) latitude, (float) longitude, 0f, System.currentTimeMillis());
            declinationDeg = field.getDeclination();
            declinationKnown = true;
        }

        // GPS course corrections only count while the robot is really driving forward.
        boolean driving =
                missionState == MissionState.RUNNING && (commandedLeft + commandedRight) / 2f > 0.05f;
        headingEstimator.addGpsFix(latitude, longitude, accuracyM, driving);
    }

    /** True if the gyro heading is valid and fresh (always true in simulation). */
    private boolean headingSensorsOk() {
        if (isSimulatedGps()) {
            return true;
        }
        return gyroHeading != null && gyroHeading.hasFreshHeading(HEADING_STALE_MS);
    }

    /** Passes the latest gyro and compass readings to the heading estimator. */
    private void feedHeadingSensors() {
        if (isSimulatedGps()) {
            return;
        }
        if (gyroHeading != null && gyroHeading.hasFreshHeading(HEADING_STALE_MS)) {
            headingEstimator.addGyro(gyroHeading.getHeadingDeg());
        }
        if (compassHeading != null
                && declinationKnown
                && compassHeading.hasFreshHeading(HEADING_STALE_MS)) {
            // The absolute sensor reports counter-clockwise from east; a compass bearing is clockwise
            // from north. Then magnetic north is corrected to true north.
            double magneticBearing = HeadingEstimator.normalize360(90.0 - compassHeading.getHeadingDeg());
            headingEstimator.addCompass(HeadingEstimator.normalize360(magneticBearing + declinationDeg));
        }
    }

    // ---------------------------------------------------------------------------
    // Mission commands
    // ---------------------------------------------------------------------------

    /** A start that cannot happen: stops everything and reports the reason. */
    private void failStart(String zone, String code) {
        stopVehicle();
        stopMissionSources();
        resetMissionRuntime();
        updateMissionZone(zone);
        missionState = MissionState.ERROR;
        missionError = code;
        Timber.e("Mission refused: %s", code);
        emitMissionStatus();
    }

    private void startMission() {
        if (!fragmentActive || vehicle == null) {
            Timber.w("Ignoring mission START: fragment not active or vehicle not ready");
            return;
        }

        // A mission cannot start while a route is being taught. The refusal is reported through the
        // normal mission status; it deliberately does NOT touch the motors, because the operator
        // may be driving manually.
        if (routeTeaching.isRecording()) {
            missionHandler.removeCallbacks(missionRunnable);
            missionStartTime = 0L;
            missionPausedAt = 0L;
            missionState = MissionState.ERROR;
            missionError = "RECORDING_ACTIVE";
            Timber.w("Mission refused: a route is being recorded");
            emitMissionStatus();
            return;
        }

        // Never keep a previous callback alive.
        missionHandler.removeCallbacks(missionRunnable);

        if (REQUIRE_SONAR_FOR_MISSION && !vehicle.isHasSonar()) {
            stopVehicle();
            missionStartTime = 0L;
            missionPausedAt = 0L;
            missionState = MissionState.ERROR;
            missionError = "NO_SONAR";
            Timber.e("Mission refused: sonar required but not available");
            emitMissionStatus();
            return;
        }

        String requestedZone = readRequestedZone();
        long now = System.currentTimeMillis();
        long nowRealtime = SystemClock.elapsedRealtime();

        // PAUSED -> resume at the same waypoint. Navigation uses the CURRENT GPS position, so manual
        // driving during the pause does not invalidate anything.
        boolean resume =
                missionState == MissionState.PAUSED
                        && missionRoute != null
                        && navigator != null
                        && (requestedZone.isEmpty() || requestedZone.equals(missionZone));

        if (resume) {
            // Keep progress and exclude the paused time from the timer.
            missionStartTime += now - missionPausedAt;
            missionPausedAt = 0L;
        } else {
            // Fresh start from IDLE / PREPARING / RUNNING / OBSTACLE_DETECTED / MISSION_COMPLETE /
            // ERROR / PAUSED(different zone).
            SavedRoute route = routeTeaching.getSavedRoute(requestedZone);
            if (route == null) {
                failStart(requestedZone, "NO_ROUTE");
                return;
            }
            boolean routeSimulated = "SIMULATED".equals(route.gpsSource);
            if (routeSimulated != isSimulatedGps()) {
                // Never drive a simulated route with real GPS (or the reverse): the coordinates differ.
                failStart(requestedZone, "ROUTE_SOURCE_MISMATCH");
                return;
            }

            stopMissionSources();
            resetMissionRuntime();
            missionId = UUID.randomUUID().toString();
            updateMissionZone(requestedZone);
            missionRoute = route;
            navigator = new GpsNavigator(route.waypoints, nowRealtime);
            missionTotalWaypoints = route.waypoints.size();
            missionWaypoint = 0;
            missionStartTime = now;
            missionPausedAt = 0L;
        }

        if (missionLocationSource == null) {
            String sourceError = startMissionSources();
            if (sourceError != null) {
                String zone = missionZone;
                failStart(ZONE_NOT_SELECTED.equals(zone) ? requestedZone : zone, sourceError);
                return;
            }
        }

        missionError = "";
        obstacleBelowSince = 0L;
        obstacleClearSince = 0L;
        // Grace period: a sonar that has not reported yet is measured from mission start.
        sonarWatchStartedAt = nowRealtime;
        lastStatusAt = nowRealtime;
        prepareStartedAt = nowRealtime;
        steering.reset();

        // The motors stay at zero until a good GPS fix and a heading reference are available.
        missionState = MissionState.PREPARING;
        emitMissionStatus();
        missionHandler.post(missionRunnable);
    }

    private void pauseMission() {
        if (!isMissionMoving()) {
            Timber.w("Ignoring mission PAUSE in state %s", missionState);
            return;
        }
        // State first, so any loop iteration that somehow still runs sees a non-moving state.
        missionState = MissionState.PAUSED;
        missionPausedAt = System.currentTimeMillis();
        missionHandler.removeCallbacks(missionRunnable);
        stopVehicle();
        emitMissionStatus();
    }

    /**
     * The operator is driving manually: hand control over. The mission becomes PAUSED and the
     * operator's drive command (already applied) is deliberately NOT zeroed.
     */
    private void manualOverrideMission() {
        missionState = MissionState.PAUSED;
        missionPausedAt = System.currentTimeMillis();
        missionHandler.removeCallbacks(missionRunnable);
        // The mission no longer commands the wheels (the operator does).
        commandedLeft = 0f;
        commandedRight = 0f;
        Timber.w("Mission paused: manual drive input");
        emitMissionStatus();
    }

    private void stopMission() {
        missionState = MissionState.IDLE;
        missionHandler.removeCallbacks(missionRunnable);
        stopVehicle();
        stopMissionSources();
        resetMissionRuntime();
        emitMissionStatus();
    }

    /** Fail-safe stop for a fault. The mission stays in ERROR until STOP or a new START. */
    private void enterError(String code) {
        missionHandler.removeCallbacks(missionRunnable);
        stopVehicle();
        stopMissionSources();
        missionState = MissionState.ERROR;
        missionError = code;
        missionPausedAt = System.currentTimeMillis();
        Timber.e("Mission error: %s", code);
        emitMissionStatus();
    }

    /**
     * Cancels the mission loop unconditionally and returns to IDLE. Used for disconnect and
     * lifecycle events. Motors are only commanded to zero if a mission was in progress, so
     * ordinary manual driving is not affected.
     */
    private void abortMission(String reason) {
        boolean wasActive = missionState != MissionState.IDLE;
        missionState = MissionState.IDLE;
        missionHandler.removeCallbacks(missionRunnable);
        if (wasActive) {
            Timber.w("Mission aborted: %s", reason);
            stopVehicle();
        }
        stopMissionSources();
        resetMissionRuntime();
        if (wasActive) {
            emitMissionStatus();
        }
    }

    // ---------------------------------------------------------------------------
    // PREPARING and navigation. Called only from the mission loop, so everything below runs on the
    // main thread and only while the mission owns the motors.
    // ---------------------------------------------------------------------------

    /** PREPARING: wait for a good GPS fix and a heading reference. The motors stay at zero. */
    private PrepResult prepareStep(long now) {
        boolean simulated = isSimulatedGps();
        if (!simulated) {
            feedHeadingSensors();
        }

        boolean fixOk = lastGoodFixAtMs != 0L && now - lastGoodFixAtMs <= GPS_FIX_STALE_MS;

        if (fixOk && !startDistanceChecked) {
            RouteWaypoint first = missionRoute.waypoints.get(0);
            startDistanceM =
                    GeoMath.distanceMeters(lastFixLat, lastFixLon, first.latitude, first.longitude);
            startDistanceChecked = true;
            if (startDistanceM > MAX_START_DISTANCE_M) {
                navFault = "FAR_FROM_ROUTE";
                return PrepResult.FAULT;
            }
        }

        boolean sensorsOk = headingSensorsOk();

        // The compass was not consistent: assume the robot faces the way the route starts (the
        // operator placed it at the start, as when the route was taught).
        if (!simulated
                && !headingEstimator.isReady()
                && ALLOW_ROUTE_ALIGNMENT_FALLBACK
                && fixOk
                && sensorsOk
                && now - prepareStartedAt >= COMPASS_WAIT_MS
                && startDistanceM >= 0
                && startDistanceM <= ALIGNMENT_START_MAX_M) {
            headingEstimator.initFromAssumedBearing(missionRoute.waypoints.get(0).heading);
            Timber.w("No consistent compass: assuming the robot faces the route's start direction");
        }

        boolean headingReady = simulated || (headingEstimator.isReady() && sensorsOk);

        if (fixOk && headingReady) {
            missionState = MissionState.RUNNING;
            navigator.markProgressNow(now);
            steering.reset();
            Timber.i("Mission ready: heading source %s", headingSourceName());
            return PrepResult.READY;
        }

        if (now - prepareStartedAt > PREPARE_TIMEOUT_MS) {
            navFault = fixOk ? "HEADING_UNAVAILABLE" : "GPS_TIMEOUT";
            return PrepResult.FAULT;
        }
        return PrepResult.WAITING;
    }

    /**
     * One navigation step: GPS position and heading -> heading error (GpsNavigator) -> left/right
     * wheel commands (DifferentialSteering). On FAULT, navFault holds the error code.
     */
    private NavResult navigateStep(long now) {
        if (!isSimulatedGps()) {
            feedHeadingSensors();
            // Fail-safe: never navigate on a stale heading.
            if (!headingSensorsOk()) {
                navFault = "HEADING_TIMEOUT";
                return NavResult.FAULT;
            }
        }

        // Fail-safe: no recent good GPS fix -> hold still, and fault if it stays lost.
        boolean fixOk = lastGoodFixAtMs != 0L && now - lastGoodFixAtMs <= GPS_FIX_STALE_MS;
        if (!fixOk) {
            stopVehicle();
            navigator.markProgressNow(now);
            if (lastGoodFixAtMs == 0L || now - lastGoodFixAtMs > GPS_LOST_TIMEOUT_MS) {
                navFault = "GPS_TIMEOUT";
                return NavResult.FAULT;
            }
            return NavResult.CONTINUE;
        }

        double heading = currentHeadingBearing();
        if (Double.isNaN(heading)) {
            navFault = "HEADING_TIMEOUT";
            return NavResult.FAULT;
        }

        GpsNavigator.Step step =
                navigator.update(lastFixLat, lastFixLon, lastFixAccuracyM, heading, now);
        missionWaypoint = step.reachedCount;
        navDistanceM = step.distanceM;
        navHeadingErrorDeg = step.headingErrorDeg;

        if (step.noProgress) {
            navFault = "NO_PROGRESS";
            return NavResult.FAULT;
        }
        if (step.complete) {
            return NavResult.ROUTE_COMPLETE;
        }

        DifferentialSteering.Command command = steering.compute(step.headingErrorDeg);
        driveVehicle(command.left, command.right);
        return NavResult.CONTINUE;
    }

    private final Runnable missionRunnable =
            new Runnable() {
                @Override
                public void run() {
                    // Guard: a queued iteration must never drive unless the mission is active and the
                    // fragment is still active.
                    if (!isMissionMoving() || !fragmentActive || !isAdded() || vehicle == null) {
                        missionHandler.removeCallbacks(this);
                        stopVehicle();
                        return;
                    }

                    long now = SystemClock.elapsedRealtime();
                    boolean statusDirty = false;

                    boolean sonarPresent = vehicle.isHasSonar();

                    // 1. Fail-safe: the firmware reports a sonar but it has gone silent.
                    if (sonarPresent) {
                        long reference = Math.max(lastSonarMessageAt, sonarWatchStartedAt);
                        if (now - reference > SONAR_STALE_MS) {
                            enterError("SONAR_TIMEOUT");
                            return;
                        }
                    }

                    // 2. Obstacle evaluation. A reading <= 0 is treated as "no valid echo": it is not an
                    // obstacle, but it also does not count as clear for leaving OBSTACLE_DETECTED.
                    float reading = vehicle.getSonarReading();
                    boolean valid = sonarPresent && reading > 0f;
                    boolean tooClose = valid && reading < OBSTACLE_STOP_CM;
                    boolean clear = valid && reading >= OBSTACLE_CLEAR_CM;

                    if (missionState == MissionState.PREPARING) {
                        // Waiting for GPS and a heading reference: the motors stay at zero.
                        stopVehicle();
                        PrepResult prepared = prepareStep(now);
                        if (prepared == PrepResult.FAULT) {
                            enterError(navFault);
                            return;
                        }
                        if (prepared == PrepResult.READY) {
                            statusDirty = true;
                        }
                    } else if (missionState == MissionState.RUNNING) {
                        if (tooClose) {
                            // Withhold motion immediately; the state change is debounced. The no-progress
                            // watchdog does not run while the robot is held.
                            stopVehicle();
                            navigator.markProgressNow(now);
                            if (obstacleBelowSince == 0L) {
                                obstacleBelowSince = now;
                            }
                            if (now - obstacleBelowSince >= OBSTACLE_CONFIRM_MS) {
                                missionState = MissionState.OBSTACLE_DETECTED;
                                obstacleClearSince = 0L;
                                statusDirty = true;
                                Timber.w("Obstacle detected at %.0f cm", reading);
                                // Confirmed object in the path: record it once (with cooldown).
                                recordDetection(reading, now);
                            }
                        } else {
                            obstacleBelowSince = 0L;

                            int waypointsBefore = missionWaypoint;
                            NavResult nav = navigateStep(now);

                            if (nav == NavResult.FAULT) {
                                enterError(navFault);
                                return;
                            }
                            if (missionWaypoint != waypointsBefore) {
                                statusDirty = true;
                            }

                            if (nav == NavResult.ROUTE_COMPLETE) {
                                missionState = MissionState.MISSION_COMPLETE;
                                missionPausedAt = System.currentTimeMillis();
                                stopVehicle();
                                stopMissionSources();
                                emitMissionStatus();
                                return;
                            }
                        }
                    } else {
                        // OBSTACLE_DETECTED: hold the motors at zero until the path has been clear for a while.
                        stopVehicle();
                        navigator.markProgressNow(now);
                        if (clear) {
                            if (obstacleClearSince == 0L) {
                                obstacleClearSince = now;
                            }
                            if (now - obstacleClearSince >= OBSTACLE_CLEAR_MS) {
                                missionState = MissionState.RUNNING;
                                obstacleBelowSince = 0L;
                                obstacleClearSince = 0L;
                                statusDirty = true;
                                Timber.i("Obstacle cleared, mission continues");
                            }
                        } else {
                            obstacleClearSince = 0L;
                        }
                    }

                    if (statusDirty || now - lastStatusAt >= STATUS_PERIOD_MS) {
                        lastStatusAt = now;
                        emitMissionStatus();
                    }

                    missionHandler.postDelayed(this, MISSION_LOOP_MS);
                }
            };
}