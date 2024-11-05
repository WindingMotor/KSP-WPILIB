// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.RocketSubsystem;
import frc.robot.util.PIDHelper;

public class AltitudeTarget extends PIDCommand {
    private final RocketSubsystem rocketSubsystem;
    private final PIDHelper pidHelper;
    private static final double DEFAULT_TOLERANCE = 10.0; // meters
    private static final double DEFAULT_VELOCITY_TOLERANCE = 0.5; // meters per second

    /**
     * Creates a new AltitudeTarget command using a PIDHelper for tuning.
     *
     * @param subsystem The rocket subsystem to use
     * @param pidHelper The PIDHelper to use for tuning
     */
    public AltitudeTarget(RocketSubsystem subsystem, PIDHelper pidHelper) {
        super(
            new PIDController(
                pidHelper.getP(),
                pidHelper.getI(),
                pidHelper.getD()
            ),
            subsystem::getAltitude,
            pidHelper.getSetpoint(),
            output -> {
                double throttle = Math.min(Math.max(output, 0.0), 1.0);
                subsystem.setThrottle(throttle);
                // Publish telemetry
                pidHelper.publishTelemetry(
                    pidHelper.getSetpoint() - subsystem.getAltitude(),
                    throttle
                );
            },
            subsystem
        );

        this.rocketSubsystem = subsystem;
        this.pidHelper = pidHelper;
        getController().setTolerance(
            pidHelper.getPositionTolerance(),
            pidHelper.getVelocityTolerance()
        );
    }

    /**
     * Creates a new AltitudeTarget command that will move the rocket to a target altitude.
     *
     * @param targetAltitude The target altitude in meters
     * @param subsystem The rocket subsystem to use
     * @param kP The proportional gain
     * @param kI The integral gain
     * @param kD The derivative gain
     */
    public AltitudeTarget(double targetAltitude, RocketSubsystem subsystem, 
                         double kP, double kI, double kD) {
        super(
            new PIDController(kP, kI, kD),
            subsystem::getAltitude,
            targetAltitude,
            output -> {
                double throttle = Math.min(Math.max(output, 0.0), 1.0);
                subsystem.setThrottle(throttle);
            },
            subsystem
        );

        this.rocketSubsystem = subsystem;
        this.pidHelper = null;
        getController().setTolerance(DEFAULT_TOLERANCE, DEFAULT_VELOCITY_TOLERANCE);
    }

    /**
     * Creates a new AltitudeTarget command with custom tolerances.
     *
     * @param targetAltitude The target altitude in meters
     * @param subsystem The rocket subsystem to use
     * @param kP The proportional gain
     * @param kI The integral gain
     * @param kD The derivative gain
     * @param tolerance The tolerance in meters
     * @param velocityTolerance The velocity tolerance in meters per second
     */
    public AltitudeTarget(double targetAltitude, RocketSubsystem subsystem,
                         double kP, double kI, double kD,
                         double tolerance, double velocityTolerance) {
        this(targetAltitude, subsystem, kP, kI, kD);
        getController().setTolerance(tolerance, velocityTolerance);
    }

    @Override
    public void initialize() {
        // Reset the PID controller when we start
        if (pidHelper != null) {
            // Update PID values before starting if using PIDHelper
            pidHelper.updateValues();
            getController().setPID(
                pidHelper.getP(),
                pidHelper.getI(),
                pidHelper.getD()
            );
            getController().setSetpoint(pidHelper.getSetpoint());
            getController().setTolerance(
                pidHelper.getPositionTolerance(),
                pidHelper.getVelocityTolerance()
            );
        }
        getController().reset();
    }

    @Override
    public void execute() {
        if (pidHelper != null) {
            // Update PID values during execution if using PIDHelper
            pidHelper.updateValues();
            getController().setPID(
                pidHelper.getP(),
                pidHelper.getI(),
                pidHelper.getD()
            );
            getController().setSetpoint(pidHelper.getSetpoint());
        }
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return false; // Never end just keep hovering
    }

    @Override
    public void end(boolean interrupted) {
        // When the command ends, set throttle to maintain current altitude
        if (!interrupted) {
            // Calculate throttle needed to maintain altitude (you might need to tune this)
            double maintainThrottle = 0.5; // Example value
            rocketSubsystem.setThrottle(maintainThrottle);
        } else {
            // If interrupted, cut throttle
            rocketSubsystem.setThrottle(0.0);
        }
    }
}