package frc.robot.krpc.parts;

import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.krpc.VesselSubsystem;
import krpc.client.services.SpaceCenter;

public class ProbeCorePart extends Part {
    private static final String COMMAND_MODULE = "ModuleCommand";
    private static final String SAS_MODULE = "ModuleSAS";
    private static final String TRANSMITTER_MODULE = "ModuleDataTransmitter";
    
    
    private String commandState;
    private boolean isHibernating;
    private double commSignal;
    private boolean sasEnabled;
    private String sasMode;
    private String antennaState;
    private double antennaRating;
    private boolean hibernateInWarp;

    public ProbeCorePart(String name, SpaceCenter.Part kspPart) {
        super(name, kspPart);
    }

    /**
     * Enables or disables hibernation mode.
     * @param hibernate true to hibernate, false to wake
     * @return true if operation was successful
     */
    public boolean setHibernation(boolean hibernate) {
        try {
            SpaceCenter.Module commandModule = modules.get(COMMAND_MODULE);
            if (commandModule == null) {
                System.out.println("ERROR: Could not find command module!");
                return false;
            }

            try {
                if (hibernate) {
                    commandModule.triggerEvent("Hibernate");
                } else {
                    commandModule.triggerEvent("Activate");
                }
                isHibernating = hibernate;
                System.out.println("Probe core " + (hibernate ? "hibernating" : "activated"));
                return true;
            } catch (Exception e) {
                System.out.println("Failed to set hibernation state: " + e.getMessage());
            }
            return false;
        } catch (Exception e) {
            System.out.println("Error in setHibernation(): " + e.getMessage());
            return false;
        }
    }

    /**
     * Enables or disables SAS.
     * @param enable true to enable SAS, false to disable
     * @return true if operation was successful
     */
    public boolean setSAS(boolean enable) {
        try {
            // SAS is controlled through the vessel's control system
            kspPart.getVessel().getControl().setSAS(enable);
            sasEnabled = enable;
            System.out.println("SAS " + (enable ? "enabled" : "disabled"));
            return true;
        } catch (Exception e) {
            System.out.println("Error setting SAS: " + e.getMessage());
            return false;
        }
    }

    /**
     * Sets the SAS mode.
     * @param mode The SAS mode to set
     * @return true if operation was successful
     */
    public boolean setLocalSASMode(SASMode mode) {
        try {
            SpaceCenter.SASMode kspMode = SpaceCenter.SASMode.valueOf(mode.name());
            kspPart.getVessel().getControl().setSASMode(kspMode);
            sasMode = mode.name();
            System.out.println("SAS mode set to: " + mode.name());
            return true;
        } catch (Exception e) {
            System.out.println("Error setting SAS mode: " + e.getMessage());
            return false;
        }
    }

    /**
     * Toggles control between probe and other control sources.
     * @return true if operation was successful
     */
    public boolean toggleControl() {
        try {
            SpaceCenter.Module commandModule = modules.get(COMMAND_MODULE);
            if (commandModule == null) return false;

            commandModule.triggerEvent("Toggle Control");
            System.out.println("Toggled probe control");
            return true;
        } catch (Exception e) {
            System.out.println("Error toggling control: " + e.getMessage());
            return false;
        }
    }

    @Override
    public void updateState() {
        try {
            SpaceCenter.Module commandModule = modules.get(COMMAND_MODULE);
            if (commandModule != null) {
                Map<String, String> fields = commandModule.getFields();
                commandState = fields.getOrDefault("Command State", "Unknown");
                isHibernating = Boolean.parseBoolean(fields.getOrDefault("Hibernation", "False"));
                commSignal = fields.getOrDefault("Comm Signal", "NA").equals("NA") ? 0.0 : parseDoubleOrZero(fields.getOrDefault("Comm Signal", "0"));
                hibernateInWarp = Boolean.parseBoolean(fields.getOrDefault("Hibernate in Warp", "False"));
            }

            SpaceCenter.Module transmitterModule = modules.get(TRANSMITTER_MODULE);
            if (transmitterModule != null) {
                Map<String, String> fields = transmitterModule.getFields();
                antennaState = fields.getOrDefault("Antenna State", "Unknown");
                antennaRating = parseDoubleOrZero(fields.get("Antenna Rating").replace("k", "000"));
            }

            isActive = "Operational".equals(commandState) && !isHibernating;
        } catch (Exception e) {
            System.out.println("Error updating probe core state: " + e.getMessage());
        }
    }

    // Getters
    public String getCommandState() { return commandState; }
    public boolean isHibernating() { return isHibernating; }
    public double getCommSignal() { return commSignal; }
    public boolean isSASEnabled() { return sasEnabled; }
    public String getSASMode() { return sasMode; }

    /**
     * Enum for SAS modes available in KSP
     */
    public enum SASMode {
        STABILITY_ASSIST,
        MANEUVER,
        PROGRADE,
        RETROGRADE,
        NORMAL,
        ANTI_NORMAL,
        RADIAL,
        ANTI_RADIAL,
        TARGET,
        ANTI_TARGET,
        NODE
    }

    /**
     * Sets SAS mode on the probe
     * @param mode The SAS mode to set
     * @return The command to set SAS mode
     */
    public Command setSASMode(SASMode mode) {
        return new InstantCommand(() -> {
            setLocalSASMode(mode);
            updateState();
        });
    }

    /**
     * Toggles SAS on the probe
     * @param enabled Whether to enable or disable SAS
     * @return The command to toggle SAS
     */
    public Command toggleSAS(boolean enabled) {
        return new InstantCommand(() -> {
            setSAS(enabled);
            updateState();
        });
    }

    /**
     * Sets hibernation state on the probe
     * @param hibernate Whether to hibernate or wake
     * @return The command to set hibernation
     */
    public Command setHibernationCommand(boolean hibernate) {
        return new InstantCommand(() -> {
            setHibernation(hibernate);
            updateState();
        });
    }

    /**
     * Toggles control on the probe
     * @return The command to toggle control
     */
    public Command toggleControlCommand() {
        return new InstantCommand(() -> {
            toggleControl();
            updateState();
        });
    }

    /**
     * Cycles through SAS modes
     * @return The command to cycle SAS modes
     */
    public Command cycleSASMode() {
        return new InstantCommand(() -> {
            SASMode currentMode = SASMode.valueOf(getSASMode());
            SASMode[] modes = SASMode.values();
            int nextIndex = (currentMode.ordinal() + 1) % modes.length;
            setSASMode(modes[nextIndex]);
            updateState();
        });
    }

    /**
     * Emergency stabilization sequence
     * @return The command for emergency stabilization
     */
    public Command emergencyStabilize() {
        return new InstantCommand(() -> {
            if (isHibernating()) {
                setHibernation(false);
            }
            setSAS(true);
            setSASMode(SASMode.STABILITY_ASSIST);
            updateState();
        });
    }

    // Convenience methods for common SAS modes
    public Command holdStabilityAssist() {
        return setSASMode(SASMode.STABILITY_ASSIST);
    }

    public Command holdPrograde() {
        return setSASMode(SASMode.PROGRADE);
    }

    public Command holdRetrograde() {
        return setSASMode(SASMode.RETROGRADE);
    }

    public Command holdNormal() {
        return setSASMode(SASMode.NORMAL);
    }

    public Command holdAntiNormal() {
        return setSASMode(SASMode.ANTI_NORMAL);
    }

    public Command holdRadial() {
        return setSASMode(SASMode.RADIAL);
    }

    public Command holdAntiRadial() {
        return setSASMode(SASMode.ANTI_RADIAL);
    }

    public Command holdTarget() {
        return setSASMode(SASMode.TARGET);
    }

    public Command holdAntiTarget() {
        return setSASMode(SASMode.ANTI_TARGET);
    }

    public Command holdManeuver() {
        return setSASMode(SASMode.MANEUVER);
    }

    // Combined commands for common operations
    public Command prepareForManeuver() {
        return new InstantCommand(() -> {
            if (isHibernating()) {
                setHibernation(false);
            }
            setSAS(true);
            setSASMode(SASMode.MANEUVER);
            updateState();
        });
    }

    public Command prepareForDocking() {
        return new InstantCommand(() -> {
            if (isHibernating()) {
                setHibernation(false);
            }
            setSAS(true);
            setSASMode(SASMode.TARGET);
            updateState();
        });
    }

    public Command killRotation() {
        return new InstantCommand(() -> {
            setSAS(true);
            setSASMode(SASMode.STABILITY_ASSIST);
            try {
                kspPart.getVessel().getControl().setRCS(true);
            } catch (Exception e) {
                System.out.println("Failed to enable RCS: " + e.getMessage());
            }
            updateState();
        });
    }

}