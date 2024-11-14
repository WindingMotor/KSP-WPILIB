package frc.robot.krpc;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.krpc.KRPCWrapper;
import frc.robot.krpc.parts.*;
import krpc.client.services.SpaceCenter;
import java.util.*;

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
            updateTelemetry();
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
            // Log error
        }
    }

    private Part createPartInstance(String name, SpaceCenter.Part kspPart) {
        try {
            // Create appropriate part instance based on modules or name
            if (name.contains("Engine")) {
                return new EnginePart(name, kspPart);
            }
            // Add more part types here
            
            return null; // Unknown part type
        } catch (Exception e) {
            // Log error
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
            part.updateState();
        }
    }

    private void updateTelemetry() {
        // Update SmartDashboard with part states
        parts.values().stream()
            .filter(part -> part instanceof EnginePart)
            .map(part -> (EnginePart) part)
            .forEach(engine -> {
                String prefix = "Engine/" + engine.getName() + "/";
                SmartDashboard.putNumber(prefix + "Thrust", engine.getThrust());
                SmartDashboard.putNumber(prefix + "FuelFlow", engine.getFuelFlow());
                SmartDashboard.putString(prefix + "Status", engine.getStatus());
                SmartDashboard.putNumber(prefix + "Throttle", engine.getThrottle());
            });
    }

    public Part getPart(String name) {
        return parts.get(name);
    }

    public List<Part> getPartsByType(Class<? extends Part> type) {
        return parts.values().stream()
            .filter(type::isInstance)
            .toList();
    }
}