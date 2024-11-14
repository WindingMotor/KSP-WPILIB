package frc.robot.krpc.parts;

import krpc.client.services.SpaceCenter;

public class WingletPart extends Part {
    private static final String CONTROL_SURFACE_MODULE = "ModuleControlSurface";
    
    private double authority;
    private double deployAngle;
    private boolean isDeployed;
    private double pitch;
    private double roll;
    private double yaw;

    public WingletPart(String name, SpaceCenter.Part kspPart) {
        super(name, kspPart);
    }

    @Override
    public void updateState() {
        try {
            authority = Double.parseDouble(getFieldValue(CONTROL_SURFACE_MODULE, "Authority Limiter"));
            deployAngle = Double.parseDouble(getFieldValue(CONTROL_SURFACE_MODULE, "Deploy Angle"));
            isDeployed = Boolean.parseBoolean(getFieldValue(CONTROL_SURFACE_MODULE, "Deploy"));
            pitch = Double.parseDouble(getFieldValue(CONTROL_SURFACE_MODULE, "Pitch"));
            roll = Double.parseDouble(getFieldValue(CONTROL_SURFACE_MODULE, "Roll"));
            yaw = Double.parseDouble(getFieldValue(CONTROL_SURFACE_MODULE, "Yaw"));
            isActive = isDeployed;
        } catch (Exception e) {
            // Log error
        }
    }

    public double getAuthority() { return authority; }
    public double getDeployAngle() { return deployAngle; }
    public boolean isDeployed() { return isDeployed; }
    public double getPitch() { return pitch; }
    public double getRoll() { return roll; }
    public double getYaw() { return yaw; }
}