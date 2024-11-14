package frc.robot.krpc.parts;

import krpc.client.services.SpaceCenter;

public class SolarPanelPart extends Part {
    private static final String SOLAR_PANEL_MODULE = "ModuleDeployableSolarPanel";
    
    private double energyFlow;
    private String status;
    private double sunExposure;

    public SolarPanelPart(String name, SpaceCenter.Part kspPart) {
        super(name, kspPart);
    }

    @Override
    public void updateState() {
        try {
            energyFlow = Double.parseDouble(getFieldValue(SOLAR_PANEL_MODULE, "Energy Flow"));
            status = getFieldValue(SOLAR_PANEL_MODULE, "Status");
            sunExposure = Double.parseDouble(getFieldValue(SOLAR_PANEL_MODULE, "Sun Exposure"));
            isActive = "Extended".equals(status);
        } catch (Exception e) {
            // Log error
        }
    }

    public double getEnergyFlow() { return energyFlow; }
    public String getStatus() { return status; }
    public double getSunExposure() { return sunExposure; }
}