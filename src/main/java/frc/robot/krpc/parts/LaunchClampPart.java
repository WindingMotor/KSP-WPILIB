package frc.robot.krpc.parts;

import krpc.client.services.SpaceCenter;

public class LaunchClampPart extends Part {
    private static final String LAUNCH_CLAMP_MODULE = "LaunchClamp";
    private static final String GENERATOR_MODULE = "ModuleGenerator";
    
    private boolean isReleased;
    private double efficiency;
    private boolean generatorActive;

    public LaunchClampPart(String name, SpaceCenter.Part kspPart) {
        super(name, kspPart);
    }

    @Override
    public void updateState() {
        try {
            efficiency = Double.parseDouble(getFieldValue(GENERATOR_MODULE, "Efficiency"));
            generatorActive = Boolean.parseBoolean(getFieldValue(GENERATOR_MODULE, "Generator"));
            isActive = !isReleased && generatorActive;
        } catch (Exception e) {
            // Log error
        }
    }

    public boolean isReleased() { return isReleased; }
    public double getEfficiency() { return efficiency; }
    public boolean isGeneratorActive() { return generatorActive; }
}