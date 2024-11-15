package frc.robot.krpc.parts;

import java.util.Map;
import krpc.client.services.SpaceCenter;

public class AntennaPart extends Part {
    private static final String DEPLOYABLE_MODULE = "ModuleDeployableAntenna";
    private static final String TRANSMITTER_MODULE = "ModuleDataTransmitter";
    
    private String status;
    private double antennaRating;

    public AntennaPart(String name, SpaceCenter.Part kspPart) {
        super(name, kspPart);
    }

    /**
     * Deploys the antenna.
     * @return true if deployment was successful, false otherwise
     */
    public boolean deploy() {
        try {
            System.out.println("Attempting to deploy antenna: " + name);
            
            SpaceCenter.Module antennaModule = modules.get(DEPLOYABLE_MODULE);
            if (antennaModule == null) {
                System.out.println("ERROR: Could not find antenna module!");
                return false;
            }

            try {
                antennaModule.triggerEvent("Extend Antenna");
                isActive = true;
                System.out.println("Antenna deployment successful");
                return true;
            } catch (Exception e) {
                System.out.println("Failed to deploy antenna: " + e.getMessage());
            }

            return false;
        } catch (Exception e) {
            System.out.println("Fatal error in deploy(): " + e.getMessage());
            return false;
        }
    }

    /**
     * Retracts the antenna.
     * @return true if retraction was successful, false otherwise
     */
    public boolean retract() {
        try {
            SpaceCenter.Module antennaModule = modules.get(DEPLOYABLE_MODULE);
            if (antennaModule == null) {
                return false;
            }

            antennaModule.triggerEvent("Retract Antenna");
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
            SpaceCenter.Module antennaModule = modules.get(DEPLOYABLE_MODULE);
            SpaceCenter.Module transmitterModule = modules.get(TRANSMITTER_MODULE);
            
            if (antennaModule != null && transmitterModule != null) {
                Map<String, String> antennaFields = antennaModule.getFields();
                Map<String, String> transmitterFields = transmitterModule.getFields();
                
                status = antennaFields.getOrDefault("Status", "Unknown");
                antennaRating = parseDoubleOrZero(transmitterFields.get("Antenna Rating"));
                isActive = "Extended".equals(status);

                if (isActive) {
                    System.out.println(String.format("Antenna %s - Rating: %.2f, Status: %s",
                        name, antennaRating, status));
                }
            }
        } catch (Exception e) {
            System.out.println("Error updating antenna state: " + e.getMessage());
        }
    }

    public String getStatus() { return status; }
    public double getAntennaRating() { return antennaRating; }
}