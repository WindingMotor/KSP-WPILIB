package frc.robot.krpc.parts;

import java.util.Map;
import krpc.client.services.SpaceCenter;

public class EnginePart extends Part {
    private static final String ENGINE_MODULE = "ModuleEngines";
    private static final String ENGINE_FX_MODULE = "ModuleEnginesFX";
    private static final String GIMBAL_MODULE = "ModuleGimbal";
    
    private double thrust;
    private double fuelFlow;
    private String status;
    private double throttle;
    private double thrustLimiter;
    private boolean gimbalEnabled;
    private double gimbalLimit;
    private boolean throttleEnabled;
    private double specificImpulse;

    public EnginePart(String name, SpaceCenter.Part kspPart) {
        super(name, kspPart);
    }

    /**
     * Activates the engine.
     * @return true if activation was successful, false otherwise
     */
    public boolean activate() {
        try {
            System.out.println("Attempting to activate engine: " + name);
            
            SpaceCenter.Module engineModule = getEngineModule();
            if (engineModule == null) {
                System.out.println("ERROR: Could not find engine module!");
                return false;
            }

            try {
                // Use the correct event name
                System.out.println("Triggering 'Activate Engine' event");
                engineModule.triggerEvent("Activate Engine");
                isActive = true;
                System.out.println("Engine activation successful");
                return true;
            } catch (Exception e) {
                System.out.println("Failed to activate engine: " + e.getMessage());
            }

            return false;

        } catch (Exception e) {
            System.out.println("Fatal error in activate(): " + e.getMessage());
            e.printStackTrace();
            return false;
        }
    }

    /**
     * Shuts down the engine.
     * @return true if shutdown was successful, false otherwise
     */
    public boolean shutdown() {
        try {
            SpaceCenter.Module engineModule = getEngineModule();
            if (engineModule == null) {
                return false;
            }

            // Use the corresponding shutdown event
            engineModule.triggerEvent("Shutdown Engine");
            isActive = false;
            return true;
        } catch (Exception e) {
            System.out.println("Error in shutdown(): " + e.getMessage());
            return false;
        }
    }

    /**
     * Sets the engine's thrust limiter.
     * @param limit value between 0.0 and 1.0
     * @return true if setting was successful, false otherwise
     */
    public boolean setThrustLimiter(double limit) {
        try {
            SpaceCenter.Module engineModule = getEngineModule();
            if (engineModule == null) {
                return false;
            }

            // Clamp the value between 0 and 1
            double clampedLimit = Math.min(Math.max(limit, 0.0), 1.0);
            
            // Scale from 0-1 to 0-100
            float scaledLimit = (float)(clampedLimit * 100.0);
            
            System.out.println("Setting thrust limiter to: " + scaledLimit + "%");
            engineModule.setFieldFloat("Thrust Limiter", scaledLimit);
            thrustLimiter = clampedLimit;
            return true;
        } catch (Exception e) {
            System.out.println("Error setting thrust limiter: " + e.getMessage());
            return false;
        }
    }

    /**
     * Sets the engine's throttle. Note: This affects all engines on the vessel.
     * @param throttleValue value between 0.0 and 1.0
     * @return true if setting was successful, false otherwise
     */
    public boolean setThrottle(double throttleValue) {
        try {
            // Clamp the value between 0 and 1
            double clampedThrottle = Math.min(Math.max(throttleValue, 0.0), 1.0);
            
            // Set throttle through vessel control
            kspPart.getVessel().getControl().setThrottle((float)clampedThrottle);
            return true;
        } catch (Exception e) {
            System.out.println("Error setting throttle: " + e.getMessage());
            return false;
        }
    }

    private SpaceCenter.Module getEngineModule() {
        // Try regular engine module first
        SpaceCenter.Module engineModule = modules.get(ENGINE_MODULE);
        
        // If not found, try FX engine module
        if (engineModule == null) {
            engineModule = modules.get(ENGINE_FX_MODULE);
        }
        
        return engineModule;
    }

    @Override
    public void updateState() {
        try {
            // Get the appropriate engine module
            SpaceCenter.Module engineModule = modules.get(ENGINE_MODULE);
            if (engineModule == null) {
                engineModule = modules.get(ENGINE_FX_MODULE);
            }
            
            if (engineModule != null) {
                Map<String, String> fields = engineModule.getFields();
                status = fields.getOrDefault("Status", "Unknown");
                specificImpulse = parseDoubleOrZero(fields.get("Specific Impulse"));
                fuelFlow = parseDoubleOrZero(fields.get("Fuel Flow"));
                thrust = parseDoubleOrZero(fields.get("Thrust"));
                throttleEnabled = Boolean.parseBoolean(fields.getOrDefault("Throttle", "False"));
                thrustLimiter = parseDoubleOrZero(fields.get("Thrust Limiter"));
            }

            // Update gimbal state
            SpaceCenter.Module gimbalModule = modules.get(GIMBAL_MODULE);
            if (gimbalModule != null) {
                Map<String, String> fields = gimbalModule.getFields();
                gimbalEnabled = Boolean.parseBoolean(fields.getOrDefault("Gimbal", "False"));
                gimbalLimit = parseDoubleOrZero(fields.get("Gimbal Limit"));
            }

            isActive = "Nominal".equals(status) || "Running".equals(status);
        } catch (Exception e) {
            System.out.println("Error updating engine state: " + e.getMessage());
        }
    }

    public double getThrust() { return thrust; }
    public double getFuelFlow() { return fuelFlow; }
    public String getStatus() { return status; }
    public double getThrottle() { return throttle; }
    public double getThrustLimiter() { return thrustLimiter; }
    public boolean isGimbalEnabled() { return gimbalEnabled; }
    public double getGimbalLimit() { return gimbalLimit; }
    public boolean isThrottleEnabled() { return throttleEnabled; }
    public double getSpecificImpulse() { return specificImpulse; }
    
}