package frc.robot.krpc.parts;

import krpc.client.services.SpaceCenter;

public class FairingPart extends Part {
    private static final String FAIRING_MODULE = "ModuleProceduralFairing";
    private static final String CARGO_BAY_MODULE = "ModuleCargoBay";
    
    private boolean isDeployed;

    public FairingPart(String name, SpaceCenter.Part kspPart) {
        super(name, kspPart);
    }

    @Override
    public void updateState() {
        try {
            // Fairings are typically one-time use
            isActive = !isDeployed;
        } catch (Exception e) {
            // Log error
        }
    }

    public boolean isDeployed() { return isDeployed; }
}