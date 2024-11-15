package frc.robot.krpc.parts;

import java.util.Map;

import krpc.client.services.SpaceCenter;

public class SolarPanelPart extends Part {
    private static final String SOLAR_PANEL_MODULE = "ModuleDeployableSolarPanel";
    
    private double energyFlow;
    private String status;
    private double sunExposure;

    public SolarPanelPart(String name, SpaceCenter.Part kspPart) {
        super(name, kspPart);
    }

    /**
     * Deploys the solar panel.
     * @return true if deployment was successful, false otherwise
     */
    public boolean deploy() {
        try {
            System.out.println("Attempting to deploy solar panel: " + name);
            
            SpaceCenter.Module panelModule = modules.get(SOLAR_PANEL_MODULE);
            if (panelModule == null) {
                System.out.println("ERROR: Could not find solar panel module!");
                return false;
            }

            try {
                panelModule.triggerEvent("Extend Panel");
                isActive = true;
                System.out.println("Solar panel deployment successful");
                return true;
            } catch (Exception e) {
                System.out.println("Failed to deploy solar panel: " + e.getMessage());
            }

            return false;
        } catch (Exception e) {
            System.out.println("Fatal error in deploy(): " + e.getMessage());
            return false;
        }
    }

    /**
     * Retracts the solar panel.
     * @return true if retraction was successful, false otherwise
     */
    public boolean retract() {
        try {
            SpaceCenter.Module panelModule = modules.get(SOLAR_PANEL_MODULE);
            if (panelModule == null) {
                return false;
            }

            panelModule.triggerEvent("Retract Panel");
            isActive = false;
            return true;
        } catch (Exception e) {
            System.out.println("Error in retract(): " + e.getMessage());
            return false;
        }
    }

    @Override
    public void updateState() {
        try {
            SpaceCenter.Module panelModule = modules.get(SOLAR_PANEL_MODULE);
            if (panelModule != null) {
                Map<String, String> fields = panelModule.getFields();
                
                energyFlow = parseDoubleOrZero(fields.get("Energy Flow"));
                status = fields.getOrDefault("Status", "Unknown");
                sunExposure = parseDoubleOrZero(fields.get("Sun Exposure"));
                isActive = "Extended".equals(status);

                if (isActive) {
                    System.out.println(String.format("Solar Panel %s - Energy: %.2f, Sun: %.2f%%, Status: %s",
                        name, energyFlow, sunExposure * 100, status));
                }
            }
        } catch (Exception e) {
            System.out.println("Error updating solar panel state: " + e.getMessage());
        }
    }

    public double getEnergyFlow() { return energyFlow; }
    public String getStatus() { return status; }
    public double getSunExposure() { return sunExposure; }
}