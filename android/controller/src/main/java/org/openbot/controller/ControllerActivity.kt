/*
 * Developed for the OpenBot project (https://openbot.org) by:
 *
 * Ivo Zivkov
 * izivkov@gmail.com
 *
 * Date: 2020-12-27, 10:59 p.m.
 */

package org.openbot.controller

import android.Manifest
import android.annotation.SuppressLint
import android.app.Activity
import android.app.AlertDialog
import android.content.pm.PackageManager
import android.content.res.ColorStateList
import android.graphics.Color
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.util.Log
import android.util.TypedValue
import android.view.*
import android.widget.AdapterView
import android.widget.ArrayAdapter
import android.widget.Button
import android.widget.LinearLayout
import android.widget.RelativeLayout
import android.widget.Spinner
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatDelegate
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import org.json.JSONObject
import org.openbot.controller.customComponents.ButtonWithBorder
import org.openbot.controller.customComponents.DualDriveSeekBar
import org.openbot.controller.customComponents.VideoViewWebRTC
import org.openbot.controller.databinding.ActivityFullscreenBinding
import org.openbot.controller.utils.LocalEventBus
import org.openbot.controller.utils.Utils
import java.util.Locale
import kotlin.system.exitProcess

class ControllerActivity : /*AppCompat*/
    Activity() { // for some reason AppCompatActivity gives errors in the IDE, but it does compile,
    private val PERMISSION_REQUEST_LOCATION = 101
    private val TAG = "ControllerActivity"
    private lateinit var binding: ActivityFullscreenBinding
    private lateinit var screenSelector: ScreenSelector

    // ---------------------------------------------------------------------------------------
    // Manual control screen extras (Back button + left/right value readout + route panel).
    // They are an overlay added inside the existing sliders layout; the OpenBot drive
    // controls themselves and their command path are untouched.
    // ---------------------------------------------------------------------------------------
    private var manualLeftValueView: TextView? = null
    private var manualRightValueView: TextView? = null
    private var routePanelView: View? = null
    private var routeTitleView: TextView? = null
    private var routeGpsView: TextView? = null
    private var routeWaypointsView: TextView? = null

    // ---------------------------------------------------------------------------------------
    // Route teaching (Steps 2-5). The robot owns the real state (saved zones, GPS, waypoints);
    // Phone B only shows what the robot reports in ROUTE_STATUS.
    //   routeQueryPending = QUERY sent, waiting for the saved zone list
    //   routePending      = RECORD_START sent, waiting for the robot's answer
    // ---------------------------------------------------------------------------------------
    private var routeQueryPending = false
    private var routePending = false
    private var routeRecording = false
    private var routeWaypointCount = 0
    private var routeZoneText = ""
    private var routeSavedZones: Set<String> = emptySet()
    private var routeGpsState = "NO_FIX"
    private var routeGpsAccuracy = -1.0
    private var routeGpsSource = ""
    private var routeGpsMock = false

    private val routeQueryTimeout = Runnable {
        if (routeQueryPending) {
            routeQueryPending = false
            Toast.makeText(
                this,
                "No response from the robot. Is the robot app up to date?",
                Toast.LENGTH_LONG
            ).show()
        }
    }
    private val routePendingTimeout = Runnable {
        if (routePending) {
            routePending = false
            Toast.makeText(
                this,
                "No response from the robot. Is the robot app up to date?",
                Toast.LENGTH_LONG
            ).show()
        }
    }

    // ---------------------------------------------------------------------------------------
    // Autonomous mission screen.
    //
    // The mission screen is an overlay added on top of the controller layout. The controller
    // layout is inflated exactly once and is never replaced, because its custom components
    // and ScreenSelector subscribe to the event buses once and cannot be re-created.
    // Mission values shown here come ONLY from the robot's MISSION_STATUS message; nothing
    // is invented locally. Only zones that have a saved route can be started.
    // ---------------------------------------------------------------------------------------
    private var missionView: View? = null
    private var missionStatusView: TextView? = null
    private var missionProgressView: TextView? = null
    private var missionZoneView: TextView? = null
    private var missionWaypointView: TextView? = null
    private var detectionCountView: TextView? = null
    private var missionTimerView: TextView? = null
    private var zoneSelectorView: Spinner? = null

    // null = no status received yet; STATE_LINK_DOWN = UI-only marker for a lost robot link.
    private var missionStateName: String? = null
    private var missionErrorText: String = ""
    private var missionZoneText: String = "--"
    private var missionWaypointValue: Int = 0
    private var missionTotalValue: Int = 0
    private var missionDetectionValue: Int = 0
    private var missionElapsedBaseSec: Long = 0L
    private var missionElapsedBaseAtMs: Long = 0L

    private val uiHandler = Handler(Looper.getMainLooper())
    private val missionTimerRunnable = object : Runnable {
        override fun run() {
            renderMissionTimer()
            uiHandler.postDelayed(this, 1000L)
        }
    }

    @SuppressLint("ClickableViewAccessibility")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityFullscreenBinding.inflate(layoutInflater)
        setContentView(binding.root)

        setupPermissions()

        screenSelector = ScreenSelector(binding)
        ConnectionSelector.init(this)

        AppCompatDelegate.setDefaultNightMode(AppCompatDelegate.MODE_NIGHT_FOLLOW_SYSTEM)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        createAppEventsSubscription()
        subscribeToStatusInfo()

        binding.leftDriveControl.setDirection(DualDriveSeekBar.LeftOrRight.LEFT)
        binding.rightDriveControl.setDirection(DualDriveSeekBar.LeftOrRight.RIGHT)

        screenSelector.hideControls()

        setupManualControls()
        setupRouteSetupButton()

        setupAutonomousMission()
        binding.autonomousMission.setOnClickListener {
            showMissionScreen()
        }

        hideSystemUI()

        BotDataListener.init()

        subscribe("VIDEO_PROTOCOL", ::onDataReceived)
        subscribeToMissionStatus()
        subscribeToRouteStatus()
    }

    // ----------------------------------- Manual control ------------------------------------

    private fun setupManualControls() {
        val overlay = layoutInflater.inflate(R.layout.manual_controls_overlay, null)

        manualLeftValueView = overlay.findViewById(R.id.manualLeftValue)
        manualRightValueView = overlay.findViewById(R.id.manualRightValue)
        routePanelView = overlay.findViewById(R.id.routePanel)
        routeTitleView = overlay.findViewById(R.id.routeTitleText)
        routeGpsView = overlay.findViewById(R.id.routeGpsText)
        routeWaypointsView = overlay.findViewById(R.id.routeWaypointsText)

        val backButton: Button? = overlay.findViewById(R.id.manualBackButton)
        val finishButton: Button? = overlay.findViewById(R.id.routeFinishButton)
        val cancelButton: Button? = overlay.findViewById(R.id.routeCancelButton)

        backButton?.setOnClickListener {
            onManualBackRequested()
        }
        finishButton?.setOnClickListener {
            finishRouteRecording()
        }
        cancelButton?.setOnClickListener {
            cancelRouteRecording()
        }

        // Added inside the sliders layout, so it is shown and hidden together with it.
        binding.driveModeSlidersLayout.addView(
            overlay,
            RelativeLayout.LayoutParams(
                RelativeLayout.LayoutParams.MATCH_PARENT,
                RelativeLayout.LayoutParams.MATCH_PARENT
            )
        )

        // Show exactly what the existing drive path is commanding. The callback can arrive
        // from a non-UI thread (the slider's return-to-zero task), so hop to the UI thread.
        DriveCommandReducer.onDriveValues = { right, left ->
            runOnUiThread { showManualValues(left, right) }
        }
        showManualValues(0f, 0f)
    }

    private fun showManualValues(left: Float, right: Float) {
        manualLeftValueView?.text = String.format(Locale.US, "Left   %+.2f", left)
        manualRightValueView?.text = String.format(Locale.US, "Right  %+.2f", right)
    }

    /** True while one of the manual drive screens (sliders or tilt) is really on screen. */
    private fun isManualControlShown(): Boolean {
        return binding.driveModeSlidersLayout.isShown || binding.controlModeTiltLayout.isShown
    }

    /** Back button / back gesture on a manual drive screen. */
    private fun onManualBackRequested() {
        if (routeRecording) {
            showLeaveRecordingDialog()
        } else {
            leaveManualControl()
        }
    }

    /**
     * Leaves the manual drive screens and returns to the main controller menu. The robot is
     * explicitly told to stop (r = 0, l = 0) and the sliders go back to their home position.
     */
    private fun leaveManualControl() {
        DriveCommandReducer.sendExplicitStop()
        binding.leftDriveControl.progress = 50
        binding.rightDriveControl.progress = 50
        screenSelector.showMainMenu()
    }

    /** Starts normal manual driving (same as "Use controls to drive"). */
    private fun startManualDriving() {
        LocalEventBus.onNext(LocalEventBus.ProgressEvents.SlidersControl)
    }

    // ------------------------------------ Route teaching -----------------------------------

    /** Adds the "Route Setup" entry next to the existing manual control buttons. */
    private fun setupRouteSetupButton() {
        val container = binding.controlModeSliders.parent as? ViewGroup ?: return
        val density = resources.displayMetrics.density

        val button = ButtonWithBorder(
            this,
            null,
            com.google.android.material.R.attr.materialButtonOutlinedStyle
        )
        button.text = "ROUTE SETUP"
        button.backgroundTintList = ColorStateList.valueOf(Color.rgb(0x40, 0x40, 0x41))
        val padding = (15 * density).toInt()
        button.setPadding(padding, padding, padding, padding)

        val margin = (20 * density).toInt()
        val params = LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT,
            LinearLayout.LayoutParams.WRAP_CONTENT
        )
        params.setMargins(margin, margin, margin, margin)

        button.setOnClickListener {
            showSaveRouteDialog()
        }
        container.addView(button, params)
    }

    private fun showSaveRouteDialog() {
        if (routeQueryPending || routePending || routeRecording) return

        AlertDialog.Builder(this)
            .setTitle("Do you want to save this route?")
            .setPositiveButton("SAVE ROUTE") { _, _ -> requestSavedZonesThenChooseZone() }
            .setNegativeButton("DON'T SAVE") { _, _ ->
                // Normal manual driving. No route command is sent, so no zone is created or changed.
                startManualDriving()
            }
            .show()
    }

    /** Asks the robot which zones are saved, so the zone chooser shows the robot's real state. */
    private fun requestSavedZonesThenChooseZone() {
        if (!sendRouteCommand("QUERY")) {
            Toast.makeText(this, "Not connected to the robot", Toast.LENGTH_LONG).show()
            return
        }
        routeQueryPending = true
        uiHandler.removeCallbacks(routeQueryTimeout)
        uiHandler.postDelayed(routeQueryTimeout, ROUTE_ANSWER_TIMEOUT_MS)
    }

    private fun showZoneChooser() {
        val labels = ROUTE_ZONES.map { zone ->
            if (routeSavedZones.contains(zone)) "$zone (saved)" else zone
        }.toTypedArray()

        AlertDialog.Builder(this)
            .setTitle("Which zone should contain this route?")
            .setItems(labels) { _, which -> onZoneChosen(ROUTE_ZONES[which]) }
            .setNegativeButton("CANCEL", null)
            .show()
    }

    private fun onZoneChosen(zone: String) {
        if (routeSavedZones.contains(zone)) {
            AlertDialog.Builder(this)
                .setTitle("$zone already exists.")
                .setMessage("The new route replaces the saved one when you finish recording. If you cancel the recording, the saved route is kept.")
                .setPositiveButton("OVERWRITE") { _, _ -> beginRouteRecording(zone, true) }
                .setNegativeButton("CANCEL", null)
                .show()
        } else {
            beginRouteRecording(zone, false)
        }
    }

    private fun showLeaveRecordingDialog() {
        AlertDialog.Builder(this)
            .setTitle("Route recording in progress")
            .setMessage("What do you want to do with this route?")
            .setPositiveButton("FINISH ROUTE") { _, _ -> finishRouteRecording() }
            .setNegativeButton("CANCEL ROUTE") { _, _ -> cancelRouteRecording() }
            .setNeutralButton("KEEP RECORDING", null)
            .show()
    }

    /** Asks the robot to start recording. Manual driving opens only once the robot confirms. */
    private fun beginRouteRecording(zone: String, overwrite: Boolean) {
        if (!sendRouteCommand("RECORD_START", zone, overwrite)) {
            Toast.makeText(this, "Not connected to the robot", Toast.LENGTH_LONG).show()
            return
        }
        routePending = true
        uiHandler.removeCallbacks(routePendingTimeout)
        uiHandler.postDelayed(routePendingTimeout, ROUTE_ANSWER_TIMEOUT_MS)
    }

    private fun finishRouteRecording() {
        sendRouteCommand("RECORD_STOP")
        endRouteRecordingUi()
    }

    private fun cancelRouteRecording() {
        sendRouteCommand("RECORD_CANCEL")
        endRouteRecordingUi()
    }

    private fun endRouteRecordingUi() {
        routeRecording = false
        routePending = false
        hideRoutePanel()
        leaveManualControl()
    }

    private fun sendRouteCommand(
        action: String,
        zone: String? = null,
        overwrite: Boolean = false
    ): Boolean {
        return try {
            val command = JSONObject().put("action", action)
            if (zone != null) {
                command.put("zone", zone)
                command.put("overwrite", overwrite)
            }
            val message = JSONObject().put("routeCmd", command).toString()
            ConnectionSelector.getConnection().sendMessage(message)
            true
        } catch (e: Exception) {
            Log.w(TAG, "Could not send route command $action: $e")
            false
        }
    }

    @SuppressLint("CheckResult")
    private fun subscribeToRouteStatus() {
        StatusEventBus.addSubject(STATUS_KEY_ROUTE)
        StatusEventBus.subscribe(
            "RouteStatus_${System.identityHashCode(this)}",
            STATUS_KEY_ROUTE,
            onNext = { raw -> onRouteStatus(raw) }
        )
    }

    private fun onRouteStatus(raw: String?) {
        if (raw == null) return
        val json = try {
            JSONObject(raw)
        } catch (e: Exception) {
            Log.w(TAG, "Bad route status: $raw")
            return
        }

        runOnUiThread {
            val state = json.optString("state", "IDLE")
            val error = json.optString("error", "")
            val savedZone = json.optString("saved_zone", "")
            val savedWaypoints = json.optInt("saved_waypoints", 0)
            routeZoneText = json.optString("zone", "")
            routeWaypointCount = json.optInt("waypoints", 0)
            routeGpsState = json.optString("gps_state", "NO_FIX")
            routeGpsAccuracy = json.optDouble("gps_accuracy_m", -1.0)
            routeGpsSource = json.optString("gps_source", "")
            routeGpsMock = json.optBoolean("gps_mock", false)

            // The robot's list of zones that have a saved route.
            val zones = mutableSetOf<String>()
            val savedArray = json.optJSONArray("saved_zones")
            if (savedArray != null) {
                for (i in 0 until savedArray.length()) {
                    zones.add(savedArray.optString(i))
                }
            }
            routeSavedZones = zones
            // The mission screen only offers zones that have a saved route.
            updateMissionZoneChoices()

            if (routeQueryPending) {
                // The answer to QUERY: now the zone chooser can show what is really saved.
                routeQueryPending = false
                uiHandler.removeCallbacks(routeQueryTimeout)
                showZoneChooser()
                return@runOnUiThread
            }

            if (routePending) {
                if (error.isNotEmpty()) {
                    routePending = false
                    uiHandler.removeCallbacks(routePendingTimeout)
                    Toast.makeText(this, describeRouteError(error), Toast.LENGTH_LONG).show()
                } else if (state == "RECORDING") {
                    // The robot confirmed: only now do we enter manual driving with the panel.
                    routePending = false
                    uiHandler.removeCallbacks(routePendingTimeout)
                    routeRecording = true
                    showRoutePanel()
                    startManualDriving()
                }
                return@runOnUiThread
            }

            routeRecording = state == "RECORDING"
            if (routeRecording) {
                updateRoutePanel()
            } else {
                hideRoutePanel()
            }

            if (savedZone.isNotEmpty()) {
                Toast.makeText(
                    this,
                    "Route saved to $savedZone ($savedWaypoints waypoints)",
                    Toast.LENGTH_LONG
                ).show()
            } else if (error.isNotEmpty() && error != "NOT_RECORDING") {
                Toast.makeText(this, describeRouteError(error), Toast.LENGTH_LONG).show()
            }
        }
    }

    private fun describeRouteError(code: String): String {
        return when (code) {
            "MISSION_ACTIVE" -> "Cannot record a route while a mission is running or paused. Stop the mission first."
            "ALREADY_RECORDING" -> "A route is already being recorded."
            "NOT_RECORDING" -> "No route is being recorded."
            "INVALID_ZONE" -> "That zone is not valid."
            "ZONE_EXISTS" -> "That zone already has a saved route. Choose it again and confirm OVERWRITE."
            "ROUTE_TOO_SHORT" -> "The route was too short to save (it needs at least 2 waypoints and 5 m). The previous route (if any) was kept."
            "SAVE_FAILED" -> "The route could not be written to the robot's storage. The previous route (if any) was kept."
            "NO_LOCATION_PERMISSION" -> "The robot app has no location permission. Allow location for the robot app in Android settings, then try again."
            "GPS_DISABLED" -> "GPS is switched off on the robot phone. Turn on location, then try again."
            "NO_ACTIVITY" -> "The robot app is not ready. Try again."
            else -> "Route recording failed: $code"
        }
    }

    private fun showRoutePanel() {
        routePanelView?.visibility = View.VISIBLE
        updateRoutePanel()
    }

    private fun updateRoutePanel() {
        routePanelView?.visibility = View.VISIBLE
        routeTitleView?.text =
            if (routeZoneText.isNotEmpty()) "Recording route - $routeZoneText" else "Recording route"
        routeWaypointsView?.text = "Waypoints: $routeWaypointCount"

        val accuracy = if (routeGpsAccuracy >= 0) {
            String.format(Locale.US, "%.0f m", routeGpsAccuracy)
        } else {
            "?"
        }
        var gpsText: String
        val gpsColor: Int
        when (routeGpsState) {
            "OK" -> {
                gpsText = "GPS: ±$accuracy"
                gpsColor = Color.rgb(0x66, 0xBB, 0x6A)
            }
            "WEAK" -> {
                gpsText = "GPS: weak signal (±$accuracy)"
                gpsColor = Color.rgb(0xFF, 0xB7, 0x4D)
            }
            else -> {
                gpsText = "GPS: waiting for fix - stay at the start point"
                gpsColor = Color.rgb(0xFF, 0xB7, 0x4D)
            }
        }
        if (routeGpsSource == "SIMULATED") {
            gpsText += " [SIMULATED]"
        } else if (routeGpsMock) {
            gpsText += " [mock location]"
        }
        routeGpsView?.text = gpsText
        routeGpsView?.setTextColor(
            if (routeGpsSource == "SIMULATED") Color.rgb(0xFF, 0x52, 0x52) else gpsColor
        )
    }

    private fun hideRoutePanel() {
        routePanelView?.visibility = View.GONE
    }

    // ----------------------------------- Mission screen ------------------------------------

    private fun setupAutonomousMission() {
        val overlay = layoutInflater.inflate(R.layout.autonomous_mission, null)

        // Opaque, touch-blocking background so the controller underneath is hidden and
        // cannot receive touches while the mission screen is open.
        val typedValue = TypedValue()
        if (theme.resolveAttribute(android.R.attr.windowBackground, typedValue, true)) {
            if (typedValue.type >= TypedValue.TYPE_FIRST_COLOR_INT &&
                typedValue.type <= TypedValue.TYPE_LAST_COLOR_INT
            ) {
                overlay.setBackgroundColor(typedValue.data)
            } else if (typedValue.resourceId != 0) {
                overlay.setBackgroundResource(typedValue.resourceId)
            } else {
                overlay.setBackgroundColor(getColor(R.color.colorPrimaryDark))
            }
        } else {
            overlay.setBackgroundColor(getColor(R.color.colorPrimaryDark))
        }
        overlay.isClickable = true
        overlay.visibility = View.GONE

        addContentView(
            overlay,
            ViewGroup.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.MATCH_PARENT
            )
        )
        missionView = overlay

        missionStatusView = overlay.findViewById(R.id.missionStatus)
        missionProgressView = overlay.findViewById(R.id.missionProgress)
        missionZoneView = overlay.findViewById(R.id.missionZone)
        missionWaypointView = overlay.findViewById(R.id.missionWaypoint)
        detectionCountView = overlay.findViewById(R.id.detectionCount)
        missionTimerView = overlay.findViewById(R.id.missionTimer)
        zoneSelectorView = overlay.findViewById(R.id.zoneSelector)

        val startMission: Button? = overlay.findViewById(R.id.startMission)
        val pauseMission: Button? = overlay.findViewById(R.id.pauseMission)
        val stopMission: Button? = overlay.findViewById(R.id.stopMission)
        val backToController: Button? = overlay.findViewById(R.id.backToController)

        // The selected zone is only what the operator WANTS to start. The zone label on the
        // screen shows what the robot reports, so selecting a zone does not change it.
        zoneSelectorView?.onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
            override fun onItemSelected(
                parent: AdapterView<*>?,
                view: View?,
                position: Int,
                id: Long
            ) {
            }

            override fun onNothingSelected(parent: AdapterView<*>?) {}
        }
        updateMissionZoneChoices()

        startMission?.setOnClickListener {
            val spinner = zoneSelectorView
            if (spinner == null || spinner.selectedItemPosition == 0) {
                missionStatusView?.text = if (routeSavedZones.isEmpty()) {
                    "Status: No saved routes - record one in Route Setup"
                } else {
                    "Status: Select a zone"
                }
                return@setOnClickListener
            }

            val zone = spinner.selectedItem.toString()
            val msg = """{"missionCmd":{"action":"START","zone":"$zone"}}"""
            sendMissionCommand(msg)
        }

        pauseMission?.setOnClickListener {
            sendMissionCommand("""{"missionCmd":{"action":"PAUSE"}}""")
        }

        stopMission?.setOnClickListener {
            sendMissionCommand("""{"missionCmd":{"action":"STOP"}}""")
        }

        backToController?.setOnClickListener {
            hideMissionScreen()
        }

        renderMission()
    }

    /** The zone list of the mission screen: only zones that have a saved route. */
    private fun updateMissionZoneChoices() {
        val spinner = zoneSelectorView ?: return
        val previous = spinner.selectedItem as? String
        val saved = routeSavedZones.toList().sorted()
        val items = if (saved.isEmpty()) {
            listOf("No saved routes")
        } else {
            listOf("Select Zone") + saved
        }
        spinner.adapter = ArrayAdapter(
            this,
            android.R.layout.simple_spinner_dropdown_item,
            items
        )
        if (previous != null) {
            val index = items.indexOf(previous)
            if (index > 0) {
                spinner.setSelection(index)
            }
        }
    }

    private fun sendMissionCommand(message: String) {
        try {
            ConnectionSelector.getConnection().sendMessage(message)
        } catch (e: Exception) {
            Log.w(TAG, "Could not send mission command: $e")
            missionStatusView?.text = "Status: Not connected to robot"
        }
    }

    private fun showMissionScreen() {
        missionView?.visibility = View.VISIBLE
        // Refresh the list of saved zones from the robot.
        sendRouteCommand("QUERY")
        renderMission()
        uiHandler.removeCallbacks(missionTimerRunnable)
        uiHandler.post(missionTimerRunnable)
    }

    private fun hideMissionScreen() {
        uiHandler.removeCallbacks(missionTimerRunnable)
        missionView?.visibility = View.GONE
    }

    private fun isMissionScreenVisible(): Boolean = missionView?.visibility == View.VISIBLE

    @SuppressLint("CheckResult")
    private fun subscribeToMissionStatus() {
        StatusEventBus.addSubject(STATUS_KEY_MISSION)
        // Unique subscriber name so the newest activity instance always receives updates.
        StatusEventBus.subscribe(
            "MissionStatus_${System.identityHashCode(this)}",
            STATUS_KEY_MISSION,
            onNext = { raw -> onMissionStatus(raw) }
        )
    }

    private fun onMissionStatus(raw: String?) {
        if (raw == null) return
        val json = try {
            JSONObject(raw)
        } catch (e: Exception) {
            Log.w(TAG, "Bad mission status: $raw")
            return
        }

        runOnUiThread {
            missionStateName = json.optString("state", "IDLE")
            missionErrorText = json.optString("error", "")
            missionZoneText = json.optString("zone", "--")
            missionWaypointValue = json.optInt("waypoint", 0)
            missionTotalValue = json.optInt("total", 0)
            missionDetectionValue = json.optInt("detections", 0)
            missionElapsedBaseSec = json.optLong("elapsed_s", 0L)
            missionElapsedBaseAtMs = SystemClock.elapsedRealtime()
            renderMission()
        }
    }

    /** Called when the link to the robot is lost: mission values are no longer known. */
    private fun onRobotLinkLost() {
        runOnUiThread {
            missionStateName = STATE_LINK_DOWN
            renderMission()

            // The robot discards an in-progress recording when the link drops.
            uiHandler.removeCallbacks(routeQueryTimeout)
            uiHandler.removeCallbacks(routePendingTimeout)
            routeQueryPending = false
            routePending = false
            routeRecording = false
            hideRoutePanel()
        }
    }

    /** The mission clock keeps running while the robot is preparing, driving or holding. */
    private fun isMissionClockRunning(): Boolean {
        return missionStateName == "PREPARING" ||
                missionStateName == "RUNNING" ||
                missionStateName == "OBSTACLE_DETECTED"
    }

    private fun currentElapsedSeconds(): Long {
        return if (isMissionClockRunning()) {
            missionElapsedBaseSec + (SystemClock.elapsedRealtime() - missionElapsedBaseAtMs) / 1000L
        } else {
            missionElapsedBaseSec
        }
    }

    private fun describeMissionError(code: String): String {
        return when (code) {
            "SONAR_TIMEOUT" -> "sonar not responding"
            "NO_SONAR" -> "sonar not available"
            "NO_ROUTE" -> "no saved route for this zone"
            "RECORDING_ACTIVE" -> "a route is being recorded"
            "ROUTE_SOURCE_MISMATCH" -> "this route was recorded with a different GPS source (simulated vs real)"
            "FAR_FROM_ROUTE" -> "the robot is too far from the start of the route"
            "GPS_TIMEOUT" -> "no GPS fix"
            "HEADING_UNAVAILABLE" -> "no heading reference - place the robot at the route start, facing along the route"
            "HEADING_TIMEOUT" -> "heading sensor not responding"
            "NO_PROGRESS" -> "the robot is not getting closer to the next waypoint"
            "NO_LOCATION_PERMISSION" -> "the robot app has no location permission"
            "GPS_DISABLED" -> "GPS is switched off on the robot phone"
            "" -> "unknown"
            else -> code
        }
    }

    private fun renderMissionTimer() {
        val state = missionStateName
        if (state == null || state == STATE_LINK_DOWN) {
            missionTimerView?.text = "Mission Time: --:--"
            return
        }
        val total = currentElapsedSeconds()
        val minutes = total / 60
        val seconds = total % 60
        missionTimerView?.text = String.format("Mission Time: %02d:%02d", minutes, seconds)
    }

    private fun renderMission() {
        val state = missionStateName
        val hasData = state != null && state != STATE_LINK_DOWN

        missionStatusView?.text = when (state) {
            null -> "Status: Waiting for robot"
            STATE_LINK_DOWN -> "Status: Robot disconnected"
            "IDLE" -> "Status: Idle"
            "PREPARING" -> "Status: Preparing - waiting for GPS and heading"
            "RUNNING" -> "Status: Running"
            "OBSTACLE_DETECTED" -> "Status: Obstacle detected - robot stopped"
            "PAUSED" -> "Status: Paused"
            "MISSION_COMPLETE" -> "Status: Mission complete"
            "ERROR" -> "Status: Error - ${describeMissionError(missionErrorText)}"
            else -> "Status: $state"
        }

        if (hasData) {
            val percent =
                if (missionTotalValue > 0) (missionWaypointValue * 100) / missionTotalValue else 0
            missionProgressView?.text = "Progress: $percent%"
            missionZoneView?.text = "Zone: $missionZoneText"
            missionWaypointView?.text = "Waypoint: $missionWaypointValue / $missionTotalValue"
            detectionCountView?.text = "Detections: $missionDetectionValue"
        } else {
            missionProgressView?.text = "Progress: --"
            missionZoneView?.text = "Zone: --"
            missionWaypointView?.text = "Waypoint: --"
            detectionCountView?.text = "Detections: --"
        }
        renderMissionTimer()
    }

    // -------------------------------- Existing controller code -----------------------------

    @SuppressLint("CheckResult")
    private fun subscribe(subject: String, onDataReceived: (String) -> Unit) {
        StatusEventBus.addSubject(subject)
        StatusEventBus.subscribe(this.javaClass.simpleName, subject, onNext = {
            onDataReceived(it as String)
        })
    }

    private fun onDataReceived(data: String) {
        // Create the type of video view programmatically based on info from the Bot app.
        when (data) {
            "WEBRTC" -> {
                val view: VideoViewWebRTC = createView(VideoViewWebRTC(this)) as VideoViewWebRTC
                view.init()
            }
            "RTSP" -> {
                Toast.makeText(
                    this,
                    "RTSP not supported by this controller. For video, set your main app to use WebRTC.",
                    Toast.LENGTH_LONG
                ).show()
            }
        }
    }

    private fun createView(view: View): View {
        view.id = R.id.video_view

        val layoutParams = ViewGroup.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.MATCH_PARENT)
        view.layoutParams = layoutParams

        if (binding.video.childCount > 0) {
            binding.video.removeAllViews()
        }
        binding.video.addView(view)
        return view
    }

    private fun setupPermissions() {
        val permission = ContextCompat.checkSelfPermission(
            this,
            Manifest.permission.ACCESS_FINE_LOCATION
        )

        if (permission != PackageManager.PERMISSION_GRANTED) {
            Log.i(TAG, "Permission to get location denied")
            makeRequest()
        }
    }

    private fun makeRequest() {
        ActivityCompat.requestPermissions(
            this,
            arrayOf(Manifest.permission.ACCESS_FINE_LOCATION),
            PERMISSION_REQUEST_LOCATION
        )
    }

    @SuppressLint("CheckResult")
    private fun subscribeToStatusInfo() {
        StatusEventBus.addSubject("CONNECTION_ACTIVE")
        StatusEventBus.subscribe(this.javaClass.simpleName, "CONNECTION_ACTIVE", onNext = {
            if (it.toBoolean()) {
                screenSelector.showControls()
            } else {
                screenSelector.hideControls()
                binding.controlModeTiltLayout.stop()
                onRobotLinkLost()
            }
        })
    }

    @SuppressLint("CheckResult")
    private fun createAppEventsSubscription() {
        LocalEventBus.subscriber.start(
            this.javaClass.simpleName,

            {
                Log.i(TAG, "Got $it event")

                when (it) {
                    LocalEventBus.ProgressEvents.ConnectionSuccessful -> {
                        Utils.beep()
                        screenSelector.showControls()
                    }
                    LocalEventBus.ProgressEvents.ConnectionStarted -> {
                    }
                    LocalEventBus.ProgressEvents.ConnectionFailed -> {
                        screenSelector.hideControls()
                    }
                    LocalEventBus.ProgressEvents.StartAdvertising -> {
                        screenSelector.hideControls()
                    }
                    LocalEventBus.ProgressEvents.Disconnected -> {
                        screenSelector.hideControls()
                        binding.controlModeTiltLayout.stop()
                        onRobotLinkLost()
                    }
                    LocalEventBus.ProgressEvents.StopAdvertising -> {
                    }
                    LocalEventBus.ProgressEvents.TemporaryConnectionProblem -> {
                        screenSelector.hideControls()
                        ConnectionSelector.getConnection().connect(this)
                    }
                    LocalEventBus.ProgressEvents.PhoneOnTable -> {
                        screenSelector.showControls()
                    }
                    LocalEventBus.ProgressEvents.AdvertisingFailed -> {
                        screenSelector.hideControls()
                    }
                }
            },
            { throwable ->
                Log.d(
                    "EventsSubscription",
                    "Got error on subscribe: $throwable"
                )
            })
    }

    private fun hideSystemUI() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
            window.insetsController.let {
                it?.systemBarsBehavior = WindowInsetsController.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
                window.navigationBarColor = getColor(R.color.colorPrimaryDark)
                it?.hide(WindowInsets.Type.systemBars())
            }
        } else {
            @Suppress("DEPRECATION")
            window.decorView.systemUiVisibility = (
                    View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
                            or View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                            or View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                            or View.SYSTEM_UI_FLAG_FULLSCREEN
                            or View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                            or View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN)
            @Suppress("DEPRECATION")
            window.addFlags(WindowManager.LayoutParams.FLAG_TRANSLUCENT_NAVIGATION)
        }
    }

    @Suppress("DEPRECATION")
    override fun onBackPressed() {
        when {
            isMissionScreenVisible() -> hideMissionScreen()
            isManualControlShown() -> onManualBackRequested()
            else -> super.onBackPressed()
        }
    }

    @Override
    override fun onPause() {
        super.onPause()
        uiHandler.removeCallbacks(missionTimerRunnable)
        uiHandler.removeCallbacks(routeQueryTimeout)
        uiHandler.removeCallbacks(routePendingTimeout)

        // Going to the background disconnects, and the robot discards an unfinished recording.
        routeQueryPending = false
        routePending = false
        routeRecording = false
        hideRoutePanel()

        // Explicit stop first (never suppressed by the change filter), then the existing
        // OpenBot safety behavior: disconnecting makes the robot stop.
        DriveCommandReducer.sendExplicitStop()
        ConnectionSelector.getConnection().disconnect()
    }

    @Override
    override fun onResume() {
        super.onResume()
        hideSystemUI()

        ConnectionSelector.getConnection().init(this)
        ConnectionSelector.getConnection().connect(this)

        if (isMissionScreenVisible()) {
            uiHandler.removeCallbacks(missionTimerRunnable)
            uiHandler.post(missionTimerRunnable)
        }
    }

    override fun onDestroy() {
        uiHandler.removeCallbacks(missionTimerRunnable)
        uiHandler.removeCallbacks(routeQueryTimeout)
        uiHandler.removeCallbacks(routePendingTimeout)
        // Do not keep a reference to this activity in the drive command path.
        DriveCommandReducer.onDriveValues = null
        super.onDestroy()
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<String>, grantResults: IntArray
    ) {
        when (requestCode) {
            PERMISSION_REQUEST_LOCATION -> {

                if (grantResults.isEmpty() || grantResults[0] != PackageManager.PERMISSION_GRANTED) {
                    Log.i(TAG, "Permission has been denied by user")
                    finish()
                    exitProcess(0)
                } else {
                    Log.i(TAG, "Permission has been granted by user")
                }
            }
        }
    }

    companion object {
        private const val STATUS_KEY_MISSION = "MISSION_STATUS"
        private const val STATUS_KEY_ROUTE = "ROUTE_STATUS"
        private const val STATE_LINK_DOWN = "LINK_DOWN"
        private const val ROUTE_ANSWER_TIMEOUT_MS = 3000L
        private val ROUTE_ZONES = listOf("Zone A", "Zone B", "Zone C")
    }
}