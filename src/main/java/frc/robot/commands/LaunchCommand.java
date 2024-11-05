// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RocketSubsystem;

public class LaunchCommand extends Command {
    private final RocketSubsystem rocketSubsystem;
    private final double targetAltitude;
    private boolean isFinished = false;

    /**
     * Creates a new LaunchCommand.
     * @param subsystem The rocket subsystem to use
     * @param targetAltitude The target altitude in meters
     */
    public LaunchCommand(RocketSubsystem subsystem, double targetAltitude) {
        this.rocketSubsystem = subsystem;
        this.targetAltitude = targetAltitude;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        rocketSubsystem.launch();
    }

    @Override
    public void execute() {
        // Simple altitude-based throttle control
        double currentAltitude = rocketSubsystem.getAltitude();
        if (currentAltitude >= targetAltitude) {
            rocketSubsystem.setThrottle(0.0);
            isFinished = true;
        } else {
            double throttleScale = Math.min(1.0, (targetAltitude - currentAltitude) / 1000.0);
            rocketSubsystem.setThrottle(throttleScale);
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        rocketSubsystem.setThrottle(0.0);
    }
}