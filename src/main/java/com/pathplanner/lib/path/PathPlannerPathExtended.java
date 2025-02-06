package com.pathplanner.lib.path;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

public class PathPlannerPathExtended {
    @SuppressWarnings("unchecked")
    public static void toPathFile(PathPlannerPath path, String folder) throws IOException {
        JSONObject json = new JSONObject();
        json.put("version", "2025.1.0");

        // Convert waypoints
        JSONArray waypointsJson = new JSONArray();
        int waypointIndex = 0;
        for (var waypoint : path.getWaypoints()) {
            System.out.println("Processing Waypoint #" + waypointIndex + ":");
            System.out.println(
                    "  Anchor: ("
                            + waypoint.anchor().getX()
                            + ", "
                            + waypoint.anchor().getY()
                            + ")");

            if (waypoint.prevControl() != null) {
                System.out.println(
                        "  Prev Control: ("
                                + waypoint.prevControl().getX()
                                + ", "
                                + waypoint.prevControl().getY()
                                + ")");
            } else {
                System.out.println("  Prev Control: null");
            }

            if (waypoint.nextControl() != null) {
                System.out.println(
                        "  Next Control: ("
                                + waypoint.nextControl().getX()
                                + ", "
                                + waypoint.nextControl().getY()
                                + ")");
            } else {
                System.out.println("  Next Control: null");
            }
            System.out.println("-------------------");

            JSONObject waypointJson = new JSONObject();

            if (waypoint.prevControl() != null) {
                JSONObject prevControl = new JSONObject();
                prevControl.put("x", waypoint.prevControl().getX());
                prevControl.put("y", waypoint.prevControl().getY());
                waypointJson.put("prevControl", prevControl);
            } else {
                waypointJson.put("prevControl", null);
            }

            JSONObject anchor = new JSONObject();
            anchor.put("x", waypoint.anchor().getX());
            anchor.put("y", waypoint.anchor().getY());
            waypointJson.put("anchor", anchor);

            if (waypoint.nextControl() != null) {
                JSONObject nextControl = new JSONObject();
                nextControl.put("x", waypoint.nextControl().getX());
                nextControl.put("y", waypoint.nextControl().getY());
                waypointJson.put("nextControl", nextControl);
            } else {
                waypointJson.put("nextControl", null);
            }

            waypointsJson.add(waypointJson);
            waypointIndex++;
        }
        json.put("waypoints", waypointsJson);

        // Convert rotation targets
        JSONArray rotationTargetsJson = new JSONArray();
        for (var target : path.getRotationTargets()) {
            JSONObject targetJson = new JSONObject();
            targetJson.put("waypointRelativePos", target.position());
            targetJson.put("rotationDegrees", target.rotation().getDegrees());
            rotationTargetsJson.add(targetJson);
        }
        json.put("rotationTargets", rotationTargetsJson);

        // Convert point towards zones
        JSONArray pointTowardsJson = new JSONArray();
        for (var zone : path.getPointTowardsZones()) {
            JSONObject zoneJson = new JSONObject();

            JSONObject targetPos = new JSONObject();
            targetPos.put("x", zone.targetPosition().getX());
            targetPos.put("y", zone.targetPosition().getY());
            zoneJson.put("targetPoint", targetPos);

            zoneJson.put("rotationOffset", zone.rotationOffset().getDegrees());
            zoneJson.put("minWaypointRelativePos", zone.minPosition());
            zoneJson.put("maxWaypointRelativePos", zone.maxPosition());
            pointTowardsJson.add(zoneJson);
        }
        json.put("pointTowardsZones", pointTowardsJson);

        // Convert constraint zones
        JSONArray constraintZonesJson = new JSONArray();
        for (var zone : path.getConstraintZones()) {
            JSONObject zoneJson = new JSONObject();
            zoneJson.put("minWaypointRelativePos", zone.minPosition());
            zoneJson.put("maxWaypointRelativePos", zone.maxPosition());

            JSONObject constraints = new JSONObject();
            constraints.put("maxVelocity", zone.constraints().maxVelocityMPS());
            constraints.put("maxAcceleration", zone.constraints().maxAccelerationMPSSq());
            constraints.put("nominalVoltage", zone.constraints().nominalVoltageVolts());
            zoneJson.put("constraints", constraints);

            constraintZonesJson.add(zoneJson);
        }
        json.put("constraintZones", constraintZonesJson);

        json.put("folder", folder);

        // Convert event markers
        JSONArray eventMarkersJson = new JSONArray();
        for (var marker : path.getEventMarkers()) {
            JSONObject markerJson = new JSONObject();
            markerJson.put("waypointRelativePos", marker.position());

            if (marker.command() != null) {
                JSONObject commandJson = new JSONObject();
                commandJson.put("type", "sequential");
                JSONArray commands = new JSONArray();
                commands.add(marker.command().getName());
                commandJson.put("data", commands);
                markerJson.put("command", commandJson);
            }

            eventMarkersJson.add(markerJson);
        }
        json.put("eventMarkers", eventMarkersJson);

        // Convert global constraints
        JSONObject globalConstraints = new JSONObject();
        globalConstraints.put("maxVelocity", path.getGlobalConstraints().maxVelocityMPS());
        globalConstraints.put(
                "maxAcceleration", path.getGlobalConstraints().maxAccelerationMPSSq());
        globalConstraints.put("nominalVoltage", path.getGlobalConstraints().nominalVoltageVolts());
        json.put("globalConstraints", globalConstraints);

        // Convert goal end state
        JSONObject goalEndState = new JSONObject();
        goalEndState.put("velocity", path.getGoalEndState().velocityMPS());
        goalEndState.put("rotation", path.getGoalEndState().rotation().getDegrees());
        json.put("goalEndState", goalEndState);

        // Convert ideal starting state if present
        if (path.getIdealStartingState() != null) {
            JSONObject idealStartState = new JSONObject();
            idealStartState.put("velocity", path.getIdealStartingState().velocityMPS());
            idealStartState.put("rotation", path.getIdealStartingState().rotation().getDegrees());
            json.put("idealStartingState", idealStartState);
        } else {
            json.put("idealStartingState", null);
        }

        json.put("reversed", path.isReversed());
        json.put("preventFlipping", path.preventFlipping);

        // Write to file
        File pathFile = new File("src/main/deploy/pathplanner/paths/" + path.name + ".path");
        pathFile.getParentFile().mkdirs();

        try (FileWriter file = new FileWriter(pathFile)) {
            file.write(json.toJSONString());
        }
    }
}
