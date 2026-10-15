package org.openbot.app.robot.robot;

import android.content.Context;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.TimeZone;
import java.util.UUID;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.RejectedExecutionException;
import org.json.JSONException;
import org.json.JSONObject;
import timber.log.Timber;

/**
 * Append-only local store for MVP object detections (one JSON object per line).
 *
 * <p>MVP detections come from the forward ultrasonic sensor, which can only tell that something is
 * close. They are therefore stored as UNCLASSIFIED objects, never as a waste type.
 *
 * <p>Position comes from the phone GPS (accuracy of metres, see gps_accuracy_m), or from the
 * desk-test simulation (location_source SIMULATED). If no GPS fix was available, the position
 * fields are null and location_source is UNAVAILABLE.
 *
 * <p>Writing happens on a background thread and a write failure must never affect robot control.
 */
public class MissionDetectionLog {

    // TODO: make the robot id configurable once the robot registry exists.
    public static final String ROBOT_ID = "KSTR-001";
    public static final String DETECTION_TYPE_OBJECT = "OBJECT_ULTRASONIC";
    public static final String STATUS_UNCLASSIFIED = "UNCLASSIFIED";

    private static final String FILE_NAME = "detections.jsonl";

    private final File file;
    private final ExecutorService executor =
            Executors.newSingleThreadExecutor(
                    runnable -> {
                        Thread thread = new Thread(runnable, "detection-log");
                        thread.setDaemon(true);
                        return thread;
                    });

    public MissionDetectionLog(Context context) {
        this.file = new File(context.getApplicationContext().getFilesDir(), FILE_NAME);
    }

    private static void putNumber(JSONObject json, String key, double value) throws JSONException {
        if (Double.isNaN(value) || Double.isInfinite(value)) {
            json.put(key, JSONObject.NULL);
        } else {
            json.put(key, value);
        }
    }

    /**
     * Builds a detection record and queues it for writing.
     *
     * @param waypointsReached number of route waypoints already reached
     * @param distanceCm sonar distance to the object, in centimetres
     * @param robotLat robot position (NaN if unknown)
     * @param headingDeg robot heading, degrees clockwise from true north (NaN if unknown)
     * @param gpsAccuracyM accuracy radius of the position, metres (NaN if unknown)
     * @param locationSource "GPS", "SIMULATED" or "UNAVAILABLE"
     * @param objectLat estimated object position: the robot position moved forward by the sonar
     *     distance along the heading (NaN if unknown)
     * @return the record, or null if it could not be built
     */
    public JSONObject append(
            String missionId,
            String zone,
            int waypointsReached,
            float distanceCm,
            double robotLat,
            double robotLon,
            double headingDeg,
            float gpsAccuracyM,
            String locationSource,
            double objectLat,
            double objectLon) {
        final JSONObject record = new JSONObject();
        try {
            SimpleDateFormat format = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS'Z'", Locale.US);
            format.setTimeZone(TimeZone.getTimeZone("UTC"));

            record.put("detection_id", UUID.randomUUID().toString());
            record.put("robot_id", ROBOT_ID);
            record.put("mission_id", missionId);
            record.put("timestamp", format.format(new Date()));
            record.put("zone", zone);
            record.put("waypoints_reached", waypointsReached);
            record.put("distance_cm", Math.round(distanceCm));
            record.put("detection_type", DETECTION_TYPE_OBJECT);
            record.put("status", STATUS_UNCLASSIFIED);

            record.put("location_source", locationSource);
            putNumber(record, "robot_latitude", robotLat);
            putNumber(record, "robot_longitude", robotLon);
            putNumber(record, "heading_deg", headingDeg);
            putNumber(record, "gps_accuracy_m", gpsAccuracyM);
            // Estimated object position (only as accurate as the GPS accuracy above).
            putNumber(record, "latitude", objectLat);
            putNumber(record, "longitude", objectLon);
        } catch (JSONException e) {
            Timber.w(e, "Could not build detection record");
            return null;
        }

        try {
            executor.execute(
                    () -> {
                        try (FileWriter writer = new FileWriter(file, true)) {
                            writer.write(record.toString());
                            writer.write("\n");
                        } catch (IOException e) {
                            Timber.w(e, "Could not write detection record");
                        }
                    });
        } catch (RejectedExecutionException e) {
            Timber.w(e, "Detection log executor rejected the write");
        }
        return record;
    }
}