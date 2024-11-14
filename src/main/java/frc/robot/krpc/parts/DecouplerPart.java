package frc.robot.krpc.parts;

import krpc.client.services.SpaceCenter;

public class DecouplerPart extends Part {
    private static final String DECOUPLE_MODULE = "ModuleDecouple";
    
    private boolean isStaged;

    public DecouplerPart(String name, SpaceCenter.Part kspPart) {
        super(name, kspPart);
    }

    @Override
    public void updateState() {
        try {
            // Decouplers are typically one-time use
            isActive = !isStaged;
        } catch (Exception e) {
            // Log error
        }
    }

    public boolean isStaged() { return isStaged; }
}
