// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RocketSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier; 

public class ThrottleCommand extends Command {
    private final RocketSubsystem rocketSubsystem;
    private final DoubleSupplier throttleSupplier;
    private final BooleanSupplier cancelSupplier;
    private boolean wasManualCancelled = false;

    /**
     * Creates a new ThrottleCommand that continuously updates the rocket's throttle.
     *
     * @param subsystem The rocket subsystem to control
     * @param throttleSupplier A supplier that returns the desired throttle value (0.0 to 1.0)
     * @param cancelSupplier A supplier that returns true when the command should be cancelled manually
     */
    public ThrottleCommand(
            RocketSubsystem subsystem,
            DoubleSupplier throttleSupplier,
            BooleanSupplier cancelSupplier) {
        this.rocketSubsystem = subsystem;
        this.throttleSupplier = throttleSupplier;
        this.cancelSupplier = cancelSupplier;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        wasManualCancelled = false;
    }

    @Override
    public void execute() {
        // Get the current throttle value from the supplier and clamp it between 0 and 1
        double throttle = Math.min(Math.max(throttleSupplier.getAsDouble(), 0.0), 1.0);
        rocketSubsystem.setThrottle(throttle);

        // Check for manual cancellation
        if (cancelSupplier.getAsBoolean()) {
            wasManualCancelled = true;
        }
    }

    @Override
    public boolean isFinished() {
        return wasManualCancelled;
    }

    @Override
    public void end(boolean interrupted) {
        // If the command was manually cancelled or interrupted, set throttle to 0
        if (wasManualCancelled || interrupted) {
            rocketSubsystem.setThrottle(0.0);
        }
    }
}