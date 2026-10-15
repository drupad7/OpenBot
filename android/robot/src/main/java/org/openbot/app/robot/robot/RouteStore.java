package org.openbot.app.robot.robot;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Scanner;
import java.util.TimeZone;
import java.util.TreeMap;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import timber.log.Timber;

/**
 * Saves taught routes to disk, one JSON file per zone (zone_a.json, zone_b.json, ...), so they
 * survive app restarts.
 *
 * <ul>
 *   <li>A save writes a temporary file, syncs it to disk and renames it over the old file, so a
 *       crash or power loss in the middle of a save leaves the previous route intact.
 *   <li>A save only reports success if the data really reached the file.
 *   <li>Loading validates every file. A damaged or unsupported file is skipped (and logged), but
 *       never deleted.
 * </ul>
 *
 * <p>Plain Java apart from the logging, so it can be tested on a desktop.
 */
public class RouteStore {

    public static final int SCHEMA_VERSION = 1;

    private final File directory;

    public RouteStore(File directory) {
        this.directory = directory;
    }

    /** "Zone A" -> "zone_a.json". */
    static String fileNameForZone(String zone) {
        return zone.trim().toLowerCase(Locale.US).replaceAll("[^a-z0-9]+", "_") + ".json";
    }

    private File fileForZone(String zone) {
        return new File(directory, fileNameForZone(zone));
    }

    /**
     * Loads the saved route of every given zone that has a valid file.
     *
     * @return zone name to route, sorted by zone name
     */
    public Map<String, SavedRoute> loadAll(List<String> zones) {
        Map<String, SavedRoute> routes = new TreeMap<>();
        for (String zone : zones) {
            File file = fileForZone(zone);
            if (!file.isFile()) {
                continue;
            }
            try {
                SavedRoute route = fromJson(new JSONObject(readFile(file)));
                if (!route.zone.equals(zone)) {
                    throw new IllegalArgumentException(
                            "file belongs to zone '" + route.zone + "', expected '" + zone + "'");
                }
                routes.put(zone, route);
            } catch (IOException | JSONException | IllegalArgumentException e) {
                // Skipped, not deleted: the operator's data is never thrown away silently.
                Timber.e(e, "Saved route for %s is unreadable and was skipped (%s)", zone, file.getName());
            }
        }
        return routes;
    }

    /**
     * Saves a route, replacing the previous file of its zone atomically.
     *
     * @return true only if the route is safely on disk
     */
    public boolean save(SavedRoute route) {
        if (!directory.isDirectory() && !directory.mkdirs()) {
            Timber.e("Could not create the routes folder %s", directory);
            return false;
        }

        File target = fileForZone(route.zone);
        File temp = new File(directory, target.getName() + ".tmp");
        try {
            byte[] data = toJson(route).toString(2).getBytes(StandardCharsets.UTF_8);
            try (FileOutputStream stream = new FileOutputStream(temp)) {
                stream.write(data);
                stream.flush();
                // Force the bytes onto the disk before the rename makes them the "real" route.
                stream.getFD().sync();
            }
            if (!temp.renameTo(target)) {
                Timber.e("Could not move %s over %s", temp.getName(), target.getName());
                temp.delete();
                return false;
            }
            return true;
        } catch (IOException | JSONException e) {
            Timber.e(e, "Could not save the route for %s", route.zone);
            temp.delete();
            return false;
        }
    }

    private static String readFile(File file) throws IOException {
        try (InputStream stream = new FileInputStream(file)) {
            byte[] buffer = new byte[(int) Math.min(file.length(), 4L * 1024 * 1024)];
            int total = 0;
            while (total < buffer.length) {
                int read = stream.read(buffer, total, buffer.length - total);
                if (read < 0) {
                    break;
                }
                total += read;
            }
            return new String(buffer, 0, total, StandardCharsets.UTF_8);
        }
    }

    static JSONObject toJson(SavedRoute route) throws JSONException {
        SimpleDateFormat format = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss'Z'", Locale.US);
        format.setTimeZone(TimeZone.getTimeZone("UTC"));

        JSONObject json = new JSONObject();
        json.put("schema_version", SCHEMA_VERSION);
        json.put("zone", route.zone);
        json.put("saved_at", format.format(new Date(route.savedAtEpochMs)));
        json.put("saved_at_epoch_ms", route.savedAtEpochMs);
        json.put("gps_source", route.gpsSource);
        json.put("mock_location", route.mockLocation);
        json.put("length_m", Math.round(route.lengthM * 10.0) / 10.0);

        JSONArray waypoints = new JSONArray();
        for (RouteWaypoint waypoint : route.waypoints) {
            JSONObject point = new JSONObject();
            point.put("sequence", waypoint.sequence);
            point.put("latitude", waypoint.latitude);
            point.put("longitude", waypoint.longitude);
            point.put("heading", waypoint.heading);
            point.put("accuracy_m", Math.round(waypoint.accuracyM * 10.0) / 10.0);
            waypoints.put(point);
        }
        json.put("waypoints", waypoints);
        return json;
    }

    /** Parses and validates a route. Throws if anything is wrong or unsupported. */
    static SavedRoute fromJson(JSONObject json) throws JSONException {
        int version = json.getInt("schema_version");
        if (version != SCHEMA_VERSION) {
            throw new IllegalArgumentException("unsupported schema version " + version);
        }

        String zone = json.getString("zone");
        JSONArray array = json.getJSONArray("waypoints");
        if (array.length() < WaypointSampler.MIN_WAYPOINTS) {
            throw new IllegalArgumentException("too few waypoints (" + array.length() + ")");
        }

        List<RouteWaypoint> waypoints = new ArrayList<>();
        for (int i = 0; i < array.length(); i++) {
            JSONObject point = array.getJSONObject(i);
            int sequence = point.getInt("sequence");
            double latitude = point.getDouble("latitude");
            double longitude = point.getDouble("longitude");
            double heading = point.getDouble("heading");
            double accuracy = point.getDouble("accuracy_m");

            if (sequence != i) {
                throw new IllegalArgumentException("waypoint " + i + " has sequence " + sequence);
            }
            if (!(latitude >= -90 && latitude <= 90) || !(longitude >= -180 && longitude <= 180)) {
                throw new IllegalArgumentException("waypoint " + i + " has an invalid position");
            }
            if (Double.isNaN(heading) || Double.isInfinite(heading) || Double.isNaN(accuracy)) {
                throw new IllegalArgumentException("waypoint " + i + " has an invalid number");
            }
            waypoints.add(new RouteWaypoint(sequence, latitude, longitude, heading, (float) accuracy));
        }

        return new SavedRoute(
                zone,
                Collections.unmodifiableList(waypoints),
                json.optString("gps_source", ""),
                json.optBoolean("mock_location", false),
                json.optDouble("length_m", 0.0),
                json.optLong("saved_at_epoch_ms", 0L));
    }
}
