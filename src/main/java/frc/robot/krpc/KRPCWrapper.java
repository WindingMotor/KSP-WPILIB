// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.krpc;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.KSPPartType;
import krpc.client.Connection;
import krpc.client.RPCException;
import krpc.client.services.SpaceCenter;
import krpc.client.services.SpaceCenter.Flight;
import krpc.client.services.SpaceCenter.Vessel;
import krpc.client.Stream;
import krpc.client.StreamException;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

import org.javatuples.Triplet;
import org.opencv.core.Mat.Tuple3;

public class KRPCWrapper implements AutoCloseable {
    private Connection connection;
    private SpaceCenter spaceCenter;
    private Vessel activeVessel;
    private Flight flight;
    private SpaceCenter.ReferenceFrame surfaceVelocityFrame;
    private SpaceCenter.ReferenceFrame orbitalVelocityFrame;
    
    // Streams for telemetry
    private Stream<Double> altitudeStream;
    private Stream<Float> throttleStream;

    private Stream<Double> surfaceSpeedStream;
    private Stream<Double> orbitalSpeedStream;
    private Stream<Triplet<Double, Double, Double>> velocityVectorStream;

    // Debugging flag
    private final boolean kDebugging;

    /**
     * Creates a new KRPCWrapper instance and connects to the KSP server.
     * @param enableDebug If true, enables debugging mode for verbose logging.
     */
    public KRPCWrapper(boolean enableDebug) {
        this.kDebugging = enableDebug;
        try {
            if (kDebugging) {
                DriverStation.reportWarning("Initializing KRPC connection...", false);
            }
            connection = Connection.newInstance("FRC Robot Controller");
            spaceCenter = SpaceCenter.newInstance(connection);
            initializeVessel();
            setupStreams();
            try {
                initializeReferenceFrames();
                setupVelocityStreams();
            } catch (Exception e) {
                DriverStation.reportError("Failed to initialize velocity streams: " + e.getMessage(), e.getStackTrace());
            }
            if (kDebugging) {
                DriverStation.reportWarning("KRPC connection initialized successfully.", false);
            }
        } catch (Exception e) {
            DriverStation.reportError("Failed to initialize KRPC connection: " + e.getMessage(), e.getStackTrace());
        }
    }

    /**
     * Initializes the active vessel and flight data.
     */
    private void initializeVessel() {
        try {
            activeVessel = spaceCenter.getActiveVessel();
            if (activeVessel == null) {
                throw new NullPointerException("Active vessel is null.");
            }
            flight = activeVessel.flight(null);
            if (kDebugging) {
                DriverStation.reportWarning("Active vessel and flight data initialized.", false);
            }
        } catch (Exception e) {
            DriverStation.reportError("Failed to initialize vessel: " + e.getMessage(), e.getStackTrace());
        }
    }

    private void initializeReferenceFrames() throws RPCException {
        if (activeVessel == null) {
            throw new NullPointerException("Active vessel is not initialized");
        }
        
        // Get reference frame for surface-relative velocity
        // Use the body's reference frame instead of surface velocity frame
        surfaceVelocityFrame = activeVessel.getOrbit().getBody().getReferenceFrame();
        
        // Get reference frame for orbital velocity
        orbitalVelocityFrame = activeVessel.getOrbit().getBody().getNonRotatingReferenceFrame();
    }

    private void setupVelocityStreams() throws RPCException {
        Flight surfaceFlight = activeVessel.flight(surfaceVelocityFrame);
        Flight orbitalFlight = activeVessel.flight(orbitalVelocityFrame);
        
        try {
            // Stream for surface-relative speed
            surfaceSpeedStream = connection.addStream(
                surfaceFlight,
                "getSpeed"
            );
            
            // Stream for orbital speed
            orbitalSpeedStream = connection.addStream(
                orbitalFlight,
                "getSpeed"
            );
            
            // Stream for velocity vector - use the surface reference frame
            velocityVectorStream = connection.addStream(
                surfaceFlight,
                "getVelocity"
            );
            
            // Start all streams
            surfaceSpeedStream.start();
            orbitalSpeedStream.start();
            velocityVectorStream.start();
            
        } catch (StreamException e) {
            DriverStation.reportError("Failed to setup velocity streams: " + e.getMessage(), e.getStackTrace());
        }
    }

    /**
     * Sets up telemetry streams for real-time data.
     */
    private void setupStreams() {
        try {
            if (activeVessel == null) {
                throw new NullPointerException("Active vessel is not initialized.");
            }

            altitudeStream = connection.addStream(flight, "getMeanAltitude");
            // Specify Float as the type for throttle
            throttleStream = connection.addStream(activeVessel.getControl(), "getThrottle");
            
            // Start all streams
            altitudeStream.start();
            throttleStream.start();

            if (kDebugging) {
                DriverStation.reportWarning("Telemetry streams set up successfully.", false);
            }
        } catch (Exception e) {
            DriverStation.reportError("Failed to setup streams: " + e.getMessage(), e.getStackTrace());
            altitudeStream = null;
            throttleStream = null;
        }
    }

    /**
     * Sets the throttle of the vessel.
     * @param throttle Value between 0.0 and 1.0
     */
    public void setThrottle(double throttle) {
        if (activeVessel == null) {
            DriverStation.reportError("Cannot set throttle: Active vessel is null.", null);
            return;
        }
        try {
            // Convert double to float for kRPC
            float clampedThrottle = (float) Math.min(Math.max(throttle, 0.0), 1.0);
            activeVessel.getControl().setThrottle(clampedThrottle);
            if (kDebugging) {
                DriverStation.reportWarning("Throttle set to: " + clampedThrottle, false);
            }
        } catch (Exception e) {
            DriverStation.reportError("Failed to set throttle: " + e.getMessage(), e.getStackTrace());
        }
    }

    /**
     * Activates the next stage of the vessel.
     */
    public void activateNextStage() {
        if (activeVessel == null) {
            DriverStation.reportError("Cannot activate next stage: Active vessel is null.", null);
            return;
        }
        try {
            activeVessel.getControl().activateNextStage();
            if (kDebugging) {
                DriverStation.reportWarning("Activated next stage.", false);
            }
        } catch (Exception e) {
            DriverStation.reportError("Failed to activate next stage: " + e.getMessage(), e.getStackTrace());
        }
    }

    /**
     * Gets the current altitude in meters.
     * @return Current altitude in meters, or 0.0 if unavailable
     */
    public double getAltitude() {
        if (altitudeStream == null) {
            DriverStation.reportError("Altitude stream is null.", null);
            return 0.0;
        }
        try {
            double altitude = altitudeStream.get();
            if (kDebugging) {
                DriverStation.reportWarning("Altitude: " + altitude + " m", false);
            }
            return altitude;
        } catch (Exception e) {
            DriverStation.reportError("Failed to get altitude: " + e.getMessage(), e.getStackTrace());
            return 0.0;
        }
    }

    /**
     * Gets the vessel's surface-relative speed in meters per second.
     * @return Surface speed in m/s, or 0.0 if unavailable
     */
    public double getSurfaceSpeed() {
        if (surfaceSpeedStream == null) {
            DriverStation.reportError("Surface speed stream is null.", null);
            return 0.0;
        }
        try {
            return surfaceSpeedStream.get();
        } catch (Exception e) {
            DriverStation.reportError("Failed to get surface speed: " + e.getMessage(), e.getStackTrace());
            return 0.0;
        }
    }

    /**
     * Gets the vessel's orbital speed in meters per second.
     * @return Orbital speed in m/s, or 0.0 if unavailable
     */
    public double getOrbitalSpeed() {
        if (orbitalSpeedStream == null) {
            DriverStation.reportError("Orbital speed stream is null.", null);
            return 0.0;
        }
        try {
            return orbitalSpeedStream.get();
        } catch (Exception e) {
            DriverStation.reportError("Failed to get orbital speed: " + e.getMessage(), e.getStackTrace());
            return 0.0;
        }
    }

/**
 * Gets the vessel's velocity vector relative to the surface.
 * @return Velocity vector as Translation3d containing (x, y, z) components in m/s
 */
public Translation3d getVelocityVector() {
    if (velocityVectorStream == null) {
        DriverStation.reportError("Velocity vector stream is null.", null);
        return new Translation3d(0, 0, 0);
    }
    try {
        // Use Triplet<Double, Double, Double> for the velocity vector
        Triplet<Double, Double, Double> velocity = velocityVectorStream.get();
        return new Translation3d(
            velocity.getValue0(),
            velocity.getValue1(),
            velocity.getValue2()
        );
    } catch (Exception e) {
        DriverStation.reportError("Failed to get velocity vector: " + e.getMessage(), e.getStackTrace());
        return new Translation3d(0, 0, 0);
    }
}

      /**
     * Gets the current throttle setting.
     * @return Current throttle value between 0.0 and 1.0, or 0.0 if unavailable
     */
    public double getThrottle() {
        if (throttleStream == null) {
            DriverStation.reportError("Throttle stream is null.", null);
            return 0.0;
        }
        try {
            // Convert Float to double explicitly
            float throttleValue = throttleStream.get();
            double throttle = (double) throttleValue;
            if (kDebugging) {
                DriverStation.reportWarning("Throttle: " + throttle, false);
            }
            return throttle;
        } catch (Exception e) {
            DriverStation.reportError("Failed to get throttle: " + e.getMessage(), e.getStackTrace());
            return 0.0;
        }
    }
    public int activatePartWithTypeAndTag(KSPPartType partType, String tag, boolean debug) {
        if (activeVessel == null) {
            DriverStation.reportError("Cannot activate parts: Active vessel is null.", null);
            return 0;
        }
        
        try {
            int activatedCount = 0;
            SpaceCenter.Parts partsCollection = activeVessel.getParts();
            java.util.List<SpaceCenter.Part> parts = partsCollection.withTag(tag);
            
            if (parts.isEmpty()) {
                DriverStation.reportError(
                    String.format("No parts found with tag '%s'", tag), null);
                return 0;
            }
    
            for (SpaceCenter.Part part : parts) {
                try {
                    java.util.List<SpaceCenter.Module> modules = part.getModules();
                    
                    if (debug) {
                        DriverStation.reportWarning(
                            String.format("Processing part '%s' with %d modules", 
                            part.getName(), modules.size()), false);
                    }
    
                    // Find the first matching module
                    SpaceCenter.Module targetModule = null;
                    for (SpaceCenter.Module module : modules) {
                        if (module.getName().equals(partType.getModuleName())) {
                            targetModule = module;
                            break;
                        }
                    }
    
                    if (targetModule == null) {
                        if (debug) {
                            StringBuilder moduleList = new StringBuilder("Available modules: ");
                            for (SpaceCenter.Module module : modules) {
                                moduleList.append(module.getName()).append(", ");
                            }
                            DriverStation.reportWarning(moduleList.toString(), false);
                        }
                        continue;
                    }
    
                    // Get available fields first
                    Map<String, String> fields = targetModule.getFields();
                    if (debug) {
                        StringBuilder fieldList = new StringBuilder("Available fields: ");
                        for (String fieldName : fields.keySet()) {
                            fieldList.append(fieldName).append(", ");
                        }
                        DriverStation.reportWarning(fieldList.toString(), false);
                    }

                    // Special handling for fairings
                    if (partType == KSPPartType.FAIRING) {
                        boolean success = false;
                        
                        // Try to find the correct field name from available fields
                        for (String fieldName : fields.keySet()) {
                            String fieldLower = fieldName.toLowerCase();
                            if (fieldLower.contains("deploy") || fieldLower.contains("jettison")) {
                                try {
                                    targetModule.setFieldBool(fieldName, true);
                                    success = true;
                                    activatedCount++;
                                    
                                    // Add time warp workaround for fairings using the correct API
                                    try {
                                        // Set physical time warp factor to 1 (lowest setting)
                                        spaceCenter.setPhysicsWarpFactor(1);
                                        Thread.sleep(100); // Brief pause
                                        spaceCenter.setPhysicsWarpFactor(0); // Return to normal time
                                    } catch (Exception warpEx) {
                                        if (debug) {
                                            DriverStation.reportWarning(
                                                "Failed to apply time warp workaround: " + warpEx.getMessage(), 
                                                false);
                                        }
                                    }
                                    
                                    break;
                                } catch (Exception e) {
                                    if (debug) {
                                        DriverStation.reportWarning(
                                            String.format("Failed to set field '%s': %s", 
                                            fieldName, e.getMessage()), false);
                                    }
                                }
                            }
                        }

                        if (!success) {
                            DriverStation.reportWarning(
                                "Could not find valid deploy/jettison field for fairing", false);
                        }
                    } else {
                        // Handle non-fairing parts
                        String actionField = partType.getActionField();
                        if (fields.containsKey(actionField)) {
                            targetModule.setFieldBool(actionField, true);
                            activatedCount++;
                        } else {
                            DriverStation.reportWarning(
                                String.format("Action field '%s' not found in available fields", 
                                actionField), false);
                        }
                    }
    
                } catch (Exception e) {
                    DriverStation.reportError("Failed to process part: " + e.getMessage(), false);
                }
            }
            
            return activatedCount;
    
        } catch (Exception e) {
            DriverStation.reportError(
                String.format("Failed to activate parts with tag %s: %s", 
                tag, e.getMessage()), false);
            return 0;
        }
    }
    
    // Overloaded method to maintain backward compatibility
    public int activatePartWithTypeAndTag(KSPPartType partType, String tag) {
        return activatePartWithTypeAndTag(partType, tag, false);
    }

/**
 * Deactivates a part with the specified type and tag.
 * @param partType The type of part to deactivate
 * @param tag The tag to search for
 * @return The number of parts deactivated
 */
public int deactivatePartWithTypeAndTag(KSPPartType partType, String tag) {
    if (activeVessel == null) {
        DriverStation.reportError("Cannot deactivate parts: Active vessel is null.", null);
        return 0;
    }
    
    try {
        int deactivatedCount = 0;
        SpaceCenter.Parts partsCollection = activeVessel.getParts();
        java.util.List<SpaceCenter.Part> parts = partsCollection.withTag(tag);
        
        for (SpaceCenter.Part part : parts) {
            java.util.List<SpaceCenter.Module> modules = part.getModules();
            
            for (SpaceCenter.Module module : modules) {
                if (module.getName().equals(partType.getModuleName())) {
                    // Skip deactivation for one-way actions
                    if (partType == KSPPartType.FAIRING || 
                        partType == KSPPartType.DECOUPLER || 
                        partType == KSPPartType.ANCHORED_DECOUPLER ||
                        partType == KSPPartType.DOCKING_PORT) {
                        DriverStation.reportWarning(
                            "Skipping deactivation of " + partType.name() + 
                            " as it's a one-way action", false);
                        continue;
                    }
                    
                    module.setFieldBool(partType.getActionField(), false);
                    deactivatedCount++;
                    if (kDebugging) {
                        DriverStation.reportWarning("Deactivated " + partType.name() + " with tag: " + tag, false);
                    }
                    break;
                }
            }
        }
        
        return deactivatedCount;
    } catch (Exception e) {
        DriverStation.reportError("Failed to deactivate parts with tag " + tag + ": " + e.getMessage(), e.getStackTrace());
        return 0;
    }
}

// Get active vessel
public SpaceCenter.Vessel getActiveVessel() {
    return activeVessel;
}

/**
 * Prints detailed information about all parts on the vessel.
 * @param debug If true, prints additional debugging information
 */
public void printVesselParts(boolean debug) {
    if (activeVessel == null) {
        DriverStation.reportError("Cannot list parts: Active vessel is null.", null);
        return;
    }

    try {
        SpaceCenter.Parts partsCollection = activeVessel.getParts();
        java.util.List<SpaceCenter.Part> allParts = partsCollection.getAll();
        
        DriverStation.reportWarning(
            String.format("=== Vessel Parts Report (%d total parts) ===", 
            allParts.size()), false);

        for (SpaceCenter.Part part : allParts) {
            try {
                String partName = part.getName();
                java.util.List<SpaceCenter.Module> modules = part.getModules();
                
                StringBuilder partInfo = new StringBuilder()
                    .append("\nPart: ").append(partName)
                    .append(" (").append(modules.size()).append(" modules)");
                DriverStation.reportWarning(partInfo.toString(), false);

                if (debug) {
                    // Print detailed module information
                    for (SpaceCenter.Module module : modules) {
                        try {
                            String moduleName = module.getName();
                            Map<String, String> fields = module.getFields();
                            
                            StringBuilder moduleInfo = new StringBuilder()
                                .append("  Module: ").append(moduleName)
                                .append("\n  Fields: ");
                            
                            // Convert map to sorted list of field names for consistent output
                            List<String> fieldNames = new ArrayList<>(fields.keySet());
                            Collections.sort(fieldNames);
                            
                            for (String fieldName : fieldNames) {
                                moduleInfo.append(fieldName).append(", ");
                            }
                            
                            DriverStation.reportWarning(moduleInfo.toString(), false);
                        } catch (Exception e) {
                            DriverStation.reportWarning(
                                "Failed to get module info: " + e.getMessage(), 
                                false);
                        }
                    }
                }
            } catch (Exception e) {
                DriverStation.reportError(
                    "Failed to process part: " + e.getMessage(), 
                    false);
            }
        }
        
        DriverStation.reportWarning("=== End Vessel Parts Report ===", false);
        
    } catch (Exception e) {
        DriverStation.reportError(
            "Failed to list vessel parts: " + e.getMessage(), 
            false);
    }
}

    @Override
    public void close() {
        try {
            if (surfaceSpeedStream != null) surfaceSpeedStream.remove();
            if (orbitalSpeedStream != null) orbitalSpeedStream.remove();
            if (velocityVectorStream != null) velocityVectorStream.remove();
        } catch (Exception e) {
            DriverStation.reportError("Failed to close streams: " + e.getMessage(), e.getStackTrace());
        }
        try {
            if (altitudeStream != null) {
                altitudeStream.remove();
                if (kDebugging) {
                    DriverStation.reportWarning("Altitude stream removed.", false);
                }
            }
            if (throttleStream != null) {
                throttleStream.remove();
                if (kDebugging) {
                    DriverStation.reportWarning("Throttle stream removed.", false);
                }
            }
            if (connection != null) {
                connection.close();
                if (kDebugging) {
                    DriverStation.reportWarning("KRPC connection closed.", false);
                }
            }
        } catch (Exception e) {
            DriverStation.reportError("Failed to close KRPC connection: " + e.getMessage(), e.getStackTrace());
        }
    }
}