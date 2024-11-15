package frc.robot.krpc.parts;

import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import krpc.client.RPCException;
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


    public String getTag() {
        try {
            return kspPart.getTag();
        } catch (RPCException e) {
            e.printStackTrace();
            return "null";
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

    protected double parseDoubleOrZero(String value) {
        try {
            if (value == null) return 0.0;
            // Remove any 'k' suffix and multiply by 1000
            if (value.endsWith("k")) {
                return Double.parseDouble(value.substring(0, value.length() - 1)) * 1000;
            }
            return Double.parseDouble(value);
        } catch (NumberFormatException e) {
            return 0.0;
        }
    }

    protected SpaceCenter.Module getModule(String moduleName) {
        return modules.get(moduleName);
    }
    
    public SpaceCenter.Part getKspPart() {
        return kspPart;
    }

    public double getMass() {
        try {
            return kspPart.getMass();
        } catch (RPCException e) {
            e.printStackTrace();
            return -1.0;
        }
    }
}