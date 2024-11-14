package frc.robot.krpc.parts;

import krpc.client.services.SpaceCenter;

public class AntennaPart extends Part {
    private static final String DEPLOYABLE_MODULE = "ModuleDeployableAntenna";
    private static final String TRANSMITTER_MODULE = "ModuleDataTransmitter";
    
    private String status;
    private double antennaRating;

    public AntennaPart(String name, SpaceCenter.Part kspPart) {
        super(name, kspPart);
    }

    @Override
    public void updateState() {
        try {
            status = getFieldValue(DEPLOYABLE_MODULE, "Status");
            antennaRating = Double.parseDouble(getFieldValue(TRANSMITTER_MODULE, "Antenna Rating"));
            isActive = "Extended".equals(status);
        } catch (Exception e) {
            // Log error
        }
    }

    public String getStatus() { return status; }
    public double getAntennaRating() { return antennaRating; }
}