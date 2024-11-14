package frc.robot.krpc.parts;

import krpc.client.services.SpaceCenter;

public class BatteryPart extends Part {
    private static final String RESOURCE_MODULE = "ModuleResource";
    
    private double chargeLevel;

    public BatteryPart(String name, SpaceCenter.Part kspPart) {
        super(name, kspPart);
    }

    @Override
    public void updateState() {
        try {
            // Battery state might need to be accessed through vessel resources
            isActive = true; // Batteries are always "active"
        } catch (Exception e) {
            // Log error
        }
    }

    public double getChargeLevel() { return chargeLevel; }
}
