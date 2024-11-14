package frc.robot.krpc.parts;

import krpc.client.services.SpaceCenter;

public class EnginePart extends Part {
    private static final String ENGINE_MODULE = "ModuleEngines";
    private static final String ENGINE_FX_MODULE = "ModuleEnginesFX";
    
    private double thrust;
    private double fuelFlow;
    private String status;
    private double throttle;

    public EnginePart(String name, SpaceCenter.Part kspPart) {
        super(name, kspPart);
    }

    @Override
    public void updateState() {
        String moduleType = modules.containsKey(ENGINE_MODULE) ? ENGINE_MODULE : ENGINE_FX_MODULE;
        
        try {
            thrust = Double.parseDouble(getFieldValue(moduleType, "Thrust"));
            fuelFlow = Double.parseDouble(getFieldValue(moduleType, "Fuel Flow"));
            status = getFieldValue(moduleType, "Status");
            throttle = Double.parseDouble(getFieldValue(moduleType, "Throttle"));
            isActive = "Nominal".equals(status);
        } catch (Exception e) {
            // Log error
        }
    }

    public double getThrust() { return thrust; }
    public double getFuelFlow() { return fuelFlow; }
    public String getStatus() { return status; }
    public double getThrottle() { return throttle; }
}