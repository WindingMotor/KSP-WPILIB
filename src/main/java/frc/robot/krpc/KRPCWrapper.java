// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.krpc;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import krpc.client.Connection;
import krpc.client.RPCException;
import krpc.client.services.SpaceCenter;
import krpc.client.services.SpaceCenter.Flight;
import krpc.client.services.SpaceCenter.Vessel;
import krpc.client.Stream;
import krpc.client.StreamException;

import java.io.IOException;

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

/**
 * Activates all parts with a specific tag.
 * @param tag The tag to search for
 * @return The number of parts activated
 */
public int activatePartWithTag(String tag) {
    if (activeVessel == null) {
        DriverStation.reportError("Cannot activate parts: Active vessel is null.", null);
        return 0;
    }
    
    try {
        int activatedCount = 0;
        // Get the parts collection first
        SpaceCenter.Parts partsCollection = activeVessel.getParts();
        // Then get the list of parts with the specified tag
        java.util.List<SpaceCenter.Part> parts = partsCollection.withTag(tag);
        
        for (SpaceCenter.Part part : parts) {
            // Get all modules for the part
            java.util.List<SpaceCenter.Module> modules = part.getModules();
            
            // Find the ModuleAirbrake module
            for (SpaceCenter.Module module : modules) {
                if (module.getName().equals("ModuleAirbrake")) {
                    module.setFieldBool("deploy", true);
                    activatedCount++;
                    break;
                }
            }
        }
        
        if (kDebugging) {
            DriverStation.reportWarning("Activated " + activatedCount + " parts with tag: " + tag, false);
        }
        return activatedCount;
    } catch (Exception e) {
        DriverStation.reportError("Failed to activate parts with tag " + tag + ": " + e.getMessage(), e.getStackTrace());
        return 0;
    }
}

/**
 * Deactivates all parts with a specific tag.
 * @param tag The tag to search for
 * @return The number of parts deactivated
 */
public int deactivatePartWithTag(String tag) {
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
                if (module.getName().equals("ModuleAirbrake")) {
                    module.setFieldBool("deploy", false);
                    deactivatedCount++;
                    break;
                }
            }
        }
        
        if (kDebugging) {
            DriverStation.reportWarning("Deactivated " + deactivatedCount + " parts with tag: " + tag, false);
        }
        return deactivatedCount;
    } catch (Exception e) {
        DriverStation.reportError("Failed to deactivate parts with tag " + tag + ": " + e.getMessage(), e.getStackTrace());
        return 0;
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