package frc.robot.krpc.parts;

import krpc.client.services.SpaceCenter;

public class FuelTankPart extends Part {
    private double fuelLevel;
    private double oxidizer;

    public FuelTankPart(String name, SpaceCenter.Part kspPart) {
        super(name, kspPart);
    }

    @Override
    public void updateState() {
        try {
            // Fuel levels might need to be accessed through vessel resources
            isActive = true; // Fuel tanks are always "active"
        } catch (Exception e) {
            // Log error
        }
    }

    public double getFuelLevel() { return fuelLevel; }
    public double getOxidizer() { return oxidizer; }
}
