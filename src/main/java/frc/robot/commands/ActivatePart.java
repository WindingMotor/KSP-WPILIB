package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RocketSubsystem;
import frc.robot.util.KSPPartType;
import edu.wpi.first.wpilibj.DriverStation;

public class ActivatePart extends Command {
    private final RocketSubsystem rocketSubsystem;
    private final String partTag;
    private final KSPPartType partType;
    private final boolean activate;
    private boolean finished = false;

    /**
     * Creates a command to activate or deactivate a specific part by type and tag.
     * @param subsystem The rocket subsystem
     * @param partType The type of part to activate/deactivate
     * @param partTag The tag of the part to activate/deactivate
     * @param activate True to activate, false to deactivate
     */
    public ActivatePart(RocketSubsystem subsystem, KSPPartType partType, String partTag, boolean activate) {
        this.rocketSubsystem = subsystem;
        this.partType = partType;
        this.partTag = partTag;
        this.activate = activate;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        try {
            if (activate) {
                rocketSubsystem.activatePartWithTypeAndTag(partType, partTag);
            } else {
                rocketSubsystem.deactivatePartWithTypeAndTag(partType, partTag);
            }
            finished = true;
        } catch (Exception e) {
            DriverStation.reportError("Failed to activate/deactivate part: " + e.getMessage(), e.getStackTrace());
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}