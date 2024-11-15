package frc.robot.krpc.parts;

import java.util.Map;
import krpc.client.services.SpaceCenter;

public class DecouplerPart extends Part {
    private static final String DECOUPLE_MODULE = "ModuleDecouple";
    private static final String ANCHORED_DECOUPLE_MODULE = "ModuleAnchoredDecoupler";
    private static final String CROSSFEED_MODULE = "ModuleToggleCrossfeed";
    private boolean isStaged;

    public DecouplerPart(String name, SpaceCenter.Part kspPart) {
        super(name, kspPart);
    }

    /**
     * Attempts to activate the decoupler.
     * @return true if decoupling was successful, false otherwise
     */
    public boolean decouple() {
        try {
            SpaceCenter.Module decoupleModule = modules.get(DECOUPLE_MODULE);
            if (decoupleModule != null) {
                decoupleModule.triggerEvent("Decouple");
                isStaged = true;
                return true;
            }
            return false;
        } catch (Exception e) {
            System.out.println("Error decoupling: " + e.getMessage());
            return false;
        }
    }

    @Override
    public void updateState() {
        try {
            // Check both regular and anchored decoupler modules
            SpaceCenter.Module decouplerModule = modules.get(DECOUPLE_MODULE);
            if (decouplerModule == null) {
                decouplerModule = modules.get(ANCHORED_DECOUPLE_MODULE);
            }

            if (decouplerModule != null) {
                try {
                    Map<String, String> fields = decouplerModule.getFields();
                    //System.out.println("\nCurrent decoupler fields:");
                    for (Map.Entry<String, String> entry : fields.entrySet()) {
                        System.out.println("- " + entry.getKey() + " = " + entry.getValue());
                    }

                    // Try to determine if it's been staged
                    String staged = fields.get("staged");
                    if (staged != null) {
                        isStaged = Boolean.parseBoolean(staged);
                    }
                } catch (Exception e) {
                    System.out.println("Error getting fields: " + e.getMessage());
                }
            }
            
            // Update active state based on staging
            isActive = !isStaged;
            
        } catch (Exception e) {
            System.out.println("Error updating decoupler state: " + e.getMessage());
        }
    }

    public boolean isStaged() {
        return isStaged;
    }

    /**
     * Gets the type of decoupler (regular or anchored)
     * @return The type of decoupler as a string
     */
    public String getDecouplerType() {
        if (modules.containsKey(DECOUPLE_MODULE)) {
            return "Regular";
        } else if (modules.containsKey(ANCHORED_DECOUPLE_MODULE)) {
            return "Anchored";
        }
        return "Unknown";
    }
}