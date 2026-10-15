package org.openbot.app.robot.robot;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Predefined waypoint routes, one per zone.
 *
 * <p>Coordinates are in metres, relative to the robot's pose at the START of a mission: the robot
 * starts at (0, 0) facing +x, and +y is to its left. The geometry below is PLACEHOLDER test geometry
 * and must be replaced with real zone routes. Zone names must match the zone names sent by Phone B.
 */
public final class MissionRoutes {

    /** One waypoint of a route. */
    public static final class Waypoint {
        public final String name;
        public final float x;
        public final float y;

        public Waypoint(String name, float x, float y) {
            this.name = name;
            this.x = x;
            this.y = y;
        }
    }

    private static final Map<String, List<Waypoint>> ROUTES = new HashMap<>();

    static {
        // Zone A: 1 m square.
        ROUTES.put(
                "Zone A",
                Collections.unmodifiableList(
                        Arrays.asList(
                                new Waypoint("A1", 1.0f, 0.0f),
                                new Waypoint("A2", 1.0f, 1.0f),
                                new Waypoint("A3", 0.0f, 1.0f),
                                new Waypoint("A4", 0.0f, 0.0f))));

        // Zone B: out, to the right, and back along an L shape.
        ROUTES.put(
                "Zone B",
                Collections.unmodifiableList(
                        Arrays.asList(
                                new Waypoint("B1", 1.5f, 0.0f),
                                new Waypoint("B2", 1.5f, -1.0f),
                                new Waypoint("B3", 0.0f, -1.0f))));

        // Zone C: straight out and back.
        ROUTES.put(
                "Zone C",
                Collections.unmodifiableList(
                        Arrays.asList(new Waypoint("C1", 2.0f, 0.0f), new Waypoint("C2", 0.0f, 0.0f))));
    }

    /** Returns the route for a zone, or null if the zone has no route. */
    public static List<Waypoint> forZone(String zone) {
        if (zone == null) {
            return null;
        }
        return ROUTES.get(zone);
    }

    private MissionRoutes() {}
}