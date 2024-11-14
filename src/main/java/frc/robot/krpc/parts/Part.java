package frc.robot.krpc.parts;

import java.util.HashMap;
import java.util.Map;
import krpc.client.services.SpaceCenter;

public abstract class Part {
    protected final String name;
    protected final SpaceCenter.Part kspPart;
    protected final Map<String, SpaceCenter.Module> modules;
    protected boolean isActive;

    public Part(String name, SpaceCenter.Part kspPart) {
        this.name = name;
        this.kspPart = kspPart;
        this.modules = new HashMap<>();
        initializeModules();
    }

    private void initializeModules() {
        try {
            for (SpaceCenter.Module module : kspPart.getModules()) {
                modules.put(module.getName(), module);
            }
        } catch (Exception e) {
            // Log error
        }
    }

    public abstract void updateState();
    
    public String getName() {
        return name;
    }

    public boolean isActive() {
        return isActive;
    }

    protected String getFieldValue(String moduleName, String fieldName) {
        try {
            SpaceCenter.Module module = modules.get(moduleName);
            if (module != null) {
                Map<String, String> fields = module.getFields();
                return fields.get(fieldName);
            }
        } catch (Exception e) {
            // Log error
        }
        return null;
    }
}