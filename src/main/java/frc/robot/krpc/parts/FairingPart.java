package frc.robot.krpc.parts;

import java.util.Map;

import krpc.client.services.SpaceCenter;

public class FairingPart extends Part {
    private static final String FAIRING_MODULE = "ModuleProceduralFairing";
    private static final String CARGO_BAY_MODULE = "ModuleCargoBay";
    private boolean isDeployed;

    public FairingPart(String name, SpaceCenter.Part kspPart) {
        super(name, kspPart);
    }

    /**
     * Attempts to deploy the fairing.
     * @return true if deployment was successful, false otherwise
     */
    public boolean deploy() {
        try {
            SpaceCenter.Module fairingModule = modules.get(FAIRING_MODULE);
            if (fairingModule != null) {
                fairingModule.triggerEvent("Deploy");
                isDeployed = true;
                return true;
            }
            return false;
        } catch (Exception e) {
            System.out.println("Error deploying fairing: " + e.getMessage());
            return false;
        }
    }

    @Override
    public void updateState() {
        try {
            SpaceCenter.Module fairingModule = modules.get(FAIRING_MODULE);
            if (fairingModule != null) {
                // Try to get the deployment state
                try {
                    Map<String, String> fields = fairingModule.getFields();
                    //System.out.println("\nCurrent fairing fields:");
                    for (Map.Entry<String, String> entry : fields.entrySet()) {
                        System.out.println("- " + entry.getKey() + " = " + entry.getValue());
                    }
                } catch (Exception e) {
                    System.out.println("Error getting fields: " + e.getMessage());
                }
            }
        } catch (Exception e) {
            System.out.println("Error updating fairing state: " + e.getMessage());
        }
    }

    public boolean isDeployed() {
        return isDeployed;
    }
}