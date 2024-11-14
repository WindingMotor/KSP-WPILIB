package frc.robot.util;

public enum KSPPartType {
    AIRBRAKE("ModuleAirbrake", "deploy"),
    PARACHUTE("ModuleParachute", "deploy"),
    ENGINE("ModuleEngines", "active"),
    ENGINE_FX("ModuleEnginesFX", "active"),
    FAIRING("ModuleProceduralFairing", "deploy"),
    DOCKING_PORT("ModuleDockingNode", "undock"),
    DECOUPLER("ModuleDecouple", "staged"),
    ANCHORED_DECOUPLER("ModuleAnchoredDecoupler", "staged"),
    SOLAR_PANEL("ModuleSolarPanel", "extended"),
    RCS("ModuleRCS", "enabled"),
    REACTION_WHEEL("ModuleReactionWheel", "active"),
    LANDING_GEAR("ModuleLandingGear", "deploy"),
    CARGO_BAY("ModuleCargoBay", "deploy");

    private final String moduleName;
    private final String actionField;

    KSPPartType(String moduleName, String actionField) {
        this.moduleName = moduleName;
        this.actionField = actionField;
    }

    public String getModuleName() {
        return moduleName;
    }

    public String getActionField() {
        return actionField;
    }
}
