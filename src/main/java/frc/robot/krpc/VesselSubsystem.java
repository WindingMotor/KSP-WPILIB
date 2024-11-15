package frc.robot.krpc;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.krpc.KRPCWrapper;
import frc.robot.krpc.parts.*;
import krpc.client.RPCException;
import krpc.client.services.SpaceCenter;
import java.util.*;
import java.util.stream.Collectors;


public class VesselSubsystem extends SubsystemBase {
    private final KRPCWrapper krpc;
    private final Map<String, Part> parts = new HashMap<>();
    private long lastUpdateTime = 0;
    private static final long UPDATE_INTERVAL_MS = 100; // Update every 100ms

    public VesselSubsystem(KRPCWrapper krpc) {
        this.krpc = krpc;
        updatePartsList();
    }

    @Override
    public void periodic() {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastUpdateTime > UPDATE_INTERVAL_MS) {
            updatePartsList();
            updatePartsState();
            displayPartsOnDashboard();
            lastUpdateTime = currentTime;
        }
    }

    private void updatePartsList() {
        try {
            SpaceCenter.Parts partsCollection = krpc.getActiveVessel().getParts();
            List<SpaceCenter.Part> kspParts = partsCollection.getAll();

            // Track new parts
            for (SpaceCenter.Part kspPart : kspParts) {
                String partName = kspPart.getName();
                if (!parts.containsKey(partName)) {
                    Part part = createPartInstance(partName, kspPart);
                    if (part != null) {
                        parts.put(partName, part);
                    }
                }
            }

            // Remove parts that no longer exist
            parts.entrySet().removeIf(entry -> !kspPartExists(entry.getKey(), kspParts));
        } catch (Exception e) {
            SmartDashboard.putString("VesselSubsystem/Error", "Failed to update parts list: " + e.getMessage());
        }
    }

    private Part createPartInstance(String name, SpaceCenter.Part kspPart) {
        try {
            String tag = kspPart.getTag();
            
            
                // Engine parts
                if (tag.startsWith("ENGINE_")) {
                    return new EnginePart(name, kspPart);
                }
                
                // Probe core parts
                if (tag.startsWith("BPROBE_")) {
                    return new ProbeCorePart(name, kspPart);
                }
                
                // Battery parts
                if (tag.startsWith("BATTERY_")) {
                    return new BatteryPart(name, kspPart);
                }
                
                // Antenna parts
                if (tag.startsWith("ANTENNA_")) {
                    return new AntennaPart(name, kspPart);
                }
                
                // Fuel tank parts
                if (tag.startsWith("TANK_")) {
                    return new FuelTankPart(name, kspPart);
                }
                
                // Decoupler parts
                if (tag.startsWith("DECOUPLER_")) {
                    return new DecouplerPart(name, kspPart);
                }
                
                // Fairing parts
                if (tag.startsWith("FAIRING_")) {
                    return new FairingPart(name, kspPart);
                }
                
                // Winglet parts
                if (tag.startsWith("WINGLET_")) {
                    return new WingletPart(name, kspPart);
                }
                
                // Solar panel parts
                if (tag.startsWith("SOLAR_")) {
                    return new SolarPanelPart(name, kspPart);
                }
            
            // If no matching tag is found, return null
            return null;
            
        } catch (Exception e) {
            System.out.println("Failed to create part instance for " + name + ": " + e.getMessage());
            return null;
        }
    }

    private boolean kspPartExists(String partName, List<SpaceCenter.Part> kspParts) {
        try {
            return kspParts.stream().anyMatch(p -> {
                try {
                    return p.getName().equals(partName);
                } catch (Exception e) {
                    return false;
                }
            });
        } catch (Exception e) {
            return false;
        }
    }

    private void updatePartsState() {
        for (Part part : parts.values()) {
            try {
                part.updateState();
            } catch (Exception e) {
                SmartDashboard.putString("VesselSubsystem/Error", 
                    "Failed to update state for " + part.getName() + ": " + e.getMessage());
            }
        }
    }

    /**
     * Displays information about all parts on the SmartDashboard.
     */
    public void displayPartsOnDashboard() {
        // Clear previous parts
        SmartDashboard.putNumber("Total Parts", parts.size());
        
        // Track parts by type
        Map<Class<? extends Part>, Integer> partTypeCounts = new HashMap<>();
        
        // Iterate through all parts
        for (Part part : parts.values()) {
            String prefix = part.getClass().getSimpleName() + "/" + part.getName() + "/";
            
            // Count parts by type
            partTypeCounts.merge(part.getClass(), 1, Integer::sum);
            
            // Add basic part information
            SmartDashboard.putBoolean(prefix + "Active", part.isActive());
            
            // Add type-specific information
            if (part instanceof EnginePart enginePart) {
                SmartDashboard.putNumber(prefix + "Thrust", enginePart.getThrust());
                SmartDashboard.putNumber(prefix + "Throttle", enginePart.getThrottle());
                SmartDashboard.putNumber(prefix + "FuelFlow", enginePart.getFuelFlow());
                SmartDashboard.putString(prefix + "Status", enginePart.getStatus());
            } else if (part instanceof ProbeCorePart probePart) {
                SmartDashboard.putString(prefix + "CommandState", probePart.getCommandState());
                SmartDashboard.putBoolean(prefix + "Hibernating", probePart.isHibernating());
                SmartDashboard.putNumber(prefix + "CommSignal", probePart.getCommSignal());
            } else if (part instanceof SolarPanelPart solarPart) {
                SmartDashboard.putNumber(prefix + "EnergyFlow", solarPart.getEnergyFlow());
                SmartDashboard.putString(prefix + "Status", solarPart.getStatus());
                SmartDashboard.putNumber(prefix + "SunExposure", solarPart.getSunExposure());
            } else if (part instanceof AntennaPart antennaPart) {
                SmartDashboard.putString(prefix + "Status", antennaPart.getStatus());
                SmartDashboard.putNumber(prefix + "Rating", antennaPart.getAntennaRating());
            } else if (part instanceof WingletPart wingletPart) {
                SmartDashboard.putBoolean(prefix + "Deployed", wingletPart.isDeployed());
                SmartDashboard.putNumber(prefix + "Authority", wingletPart.getAuthority());
                SmartDashboard.putNumber(prefix + "DeployAngle", wingletPart.getDeployAngle());
                SmartDashboard.putNumber(prefix + "Pitch", wingletPart.getPitch());
                SmartDashboard.putNumber(prefix + "Roll", wingletPart.getRoll());
                SmartDashboard.putNumber(prefix + "Yaw", wingletPart.getYaw());
            } else if (part instanceof FuelTankPart fuelTank) {
                SmartDashboard.putNumber(prefix + "FuelLevel", fuelTank.getFuelLevel());
                SmartDashboard.putNumber(prefix + "Oxidizer", fuelTank.getOxidizer());
            } else if (part instanceof DecouplerPart decoupler) {
                SmartDashboard.putBoolean(prefix + "Staged", decoupler.isStaged());
            } else if (part instanceof FairingPart fairing) {
                SmartDashboard.putBoolean(prefix + "Deployed", fairing.isDeployed());
            } else if (part instanceof LaunchClampPart clamp) {
                SmartDashboard.putBoolean(prefix + "Released", clamp.isReleased());
                SmartDashboard.putNumber(prefix + "Efficiency", clamp.getEfficiency());
                SmartDashboard.putBoolean(prefix + "GeneratorActive", clamp.isGeneratorActive());
            } else if (part instanceof BatteryPart battery) {
                SmartDashboard.putNumber(prefix + "ChargeLevel", battery.getChargeLevel());
            }
        }
        
        // Display part type counts
        for (Map.Entry<Class<? extends Part>, Integer> entry : partTypeCounts.entrySet()) {
            SmartDashboard.putNumber("PartCount/" + entry.getKey().getSimpleName(), entry.getValue());
        }

        for (Part part : parts.values()) {
            String prefix = part.getClass().getSimpleName() + "/" + part.getName() + "/";
            SmartDashboard.putString(prefix + "Tag", String.join(", ", part.getTag()));
        }
    }

    /**
     * Gets all wrapped Part objects that have the specified tag.
     * @param tag The tag to search for
     * @return List of Part objects with the specified tag
     */
    public List<Part> getPartsByTag(String tag) {
        for (Part part : parts.values()) {
            if (part.getTag().equals(tag)) {
                return Collections.singletonList(part);
            }
        }
        return Collections.emptyList();
    }

    public Part getFirstPartByTag (String tag) {
        return getPartsByTag(tag).get(0);
    }

    public void printVesselParts(boolean debug) {
        try {
            SpaceCenter.Parts partsCollection = krpc.getActiveVessel().getParts();
            List<SpaceCenter.Part> allParts = partsCollection.getAll();
            
            System.out.println("=== Vessel Parts Report (" + allParts.size() + " total parts) ===");
    
            for (SpaceCenter.Part part : allParts) {
                try {
                    String partName = part.getName();
                    List<SpaceCenter.Module> modules = part.getModules();
                    
                    // Print basic part info
                    System.out.println("\nPart: " + partName + " (" + modules.size() + " modules)");
    
                    if (debug) {
                        // Print detailed module information
                        for (SpaceCenter.Module module : modules) {
                            try {
                                String moduleName = module.getName();
                                Map<String, String> fields = module.getFields();
                                
                                System.out.println("  Module: " + moduleName);
                                if (!fields.isEmpty()) {
                                    System.out.println("  Fields: ");
                                    for (Map.Entry<String, String> field : fields.entrySet()) {
                                        System.out.println("    - " + field.getKey() + " = " + field.getValue());
                                    }
                                }
                            } catch (Exception e) {
                                System.out.println("Failed to get module info: " + e.getMessage());
                            }
                        }
    
                        // Print tags if any
                        try {
                            List<String> tags = new ArrayList<>();
                            // Since we can't directly get tags, we'll check if this part is in any tagged collections
                            if (!partsCollection.withTag(partName).isEmpty()) {
                                tags.add(partName);
                            }
                            if (!tags.isEmpty()) {
                                System.out.println("  Tags: " + String.join(", ", tags));
                            }
                        } catch (Exception e) {
                            System.out.println("Failed to get tags: " + e.getMessage());
                        }
                    }
                } catch (Exception e) {
                    System.out.println("Failed to process part: " + e.getMessage());
                }
            }
            
            System.out.println("\n=== End Vessel Parts Report ===");
            
        } catch (Exception e) {
            System.out.println("Failed to list vessel parts: " + e.getMessage());
        }
    }


}