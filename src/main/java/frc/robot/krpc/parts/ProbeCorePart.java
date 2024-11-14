package frc.robot.krpc.parts;

import krpc.client.services.SpaceCenter;

public class ProbeCorePart extends Part {
    private static final String COMMAND_MODULE = "ModuleCommand";
    private static final String SAS_MODULE = "ModuleSAS";
    
    private String commandState;
    private boolean isHibernating;
    private double commSignal;

    public ProbeCorePart(String name, SpaceCenter.Part kspPart) {
        super(name, kspPart);
    }

    @Override
    public void updateState() {
        try {
            commandState = getFieldValue(COMMAND_MODULE, "Command State");
            isHibernating = Boolean.parseBoolean(getFieldValue(COMMAND_MODULE, "Hibernation"));
            commSignal = Double.parseDouble(getFieldValue(COMMAND_MODULE, "Comm Signal"));
            isActive = !isHibernating;
        } catch (Exception e) {
            // Log error
        }
    }

    public String getCommandState() { return commandState; }
    public boolean isHibernating() { return isHibernating; }
    public double getCommSignal() { return commSignal; }
}
