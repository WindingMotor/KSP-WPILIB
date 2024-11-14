package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.RocketSubsystem;
import frc.robot.util.PIDHelper;

public class VelocityTarget extends PIDCommand {
    private final RocketSubsystem rocketSubsystem;
    private final DoubleSupplier setpointSupplier;
    private final PIDHelper pidHelper;
    private double lastSetpoint;
    private static final String AIRBRAKE_TAG = "airbrake";
    
    public VelocityTarget(RocketSubsystem subsystem, DoubleSupplier setpointSupplier, 
                         PIDHelper pidHelper) {
        super(
            new PIDController(
                pidHelper.getP(),
                pidHelper.getI(),
                pidHelper.getD()
            ),
            // Measurement supplier
            () -> subsystem.getZVelocity(),
            // Use setpoint supplier
            setpointSupplier,
            // Output consumer with proper clamping
            output -> {
                // Calculate current velocity for error
                double currentVelocity = subsystem.getZVelocity();
                
                double error = setpointSupplier.getAsDouble() - currentVelocity;
                
                // Determine if we need to slow down or speed up
                if (error < 0) { // Going too fast, need to slow down
                    // Calculate and apply airbrake deployment
                    subsystem.setThrottle(0.0); // Cut throttle when braking
                   // subsystem.activatePartWithTag(AIRBRAKE_TAG);
                } else { // Going too slow, need to speed up
                    // Calculate and clamp throttle (0 to 1)
                    double throttle = Math.min(Math.max(output, 0.0), 1.0);
                    subsystem.setThrottle(throttle);
                  //  subsystem.deactivatePartWithTag(AIRBRAKE_TAG);
                }
                
                // Publish telemetry
                pidHelper.publishTelemetry(error, output);
            },
            subsystem
        );

        this.rocketSubsystem = subsystem;
        this.setpointSupplier = setpointSupplier;
        this.pidHelper = pidHelper;
        this.lastSetpoint = setpointSupplier.getAsDouble();

        // Configure PID controller
        PIDController controller = getController();
        controller.setTolerance(
            pidHelper.getPositionTolerance(),
            pidHelper.getVelocityTolerance()
        );
        
        // Set output constraints
        controller.setIntegratorRange(-1.0, 1.0);
    }

    @Override
    public void initialize() {
        if (pidHelper != null) {
            pidHelper.updateValues();
            PIDController controller = getController();
            controller.setPID(
                pidHelper.getP(),
                pidHelper.getI(),
                pidHelper.getD()
            );
            lastSetpoint = setpointSupplier.getAsDouble();
            controller.setSetpoint(lastSetpoint);
            controller.reset();
        }
    }

    @Override
    public void execute() {
        if (pidHelper != null) {
            pidHelper.updateValues();
            PIDController controller = getController();
            
            // Get new setpoint from supplier
            double newSetpoint = setpointSupplier.getAsDouble();
            SmartDashboard.putNumber("Velocity Target Setpoint", newSetpoint);
            
            // If setpoint changed significantly, reset the controller
            if (Math.abs(newSetpoint - lastSetpoint) > 0.1) {
                controller.reset();
                lastSetpoint = newSetpoint;
            }
            
            // Update PID parameters
            controller.setPID(
                pidHelper.getP(),
                pidHelper.getI(),
                pidHelper.getD()
            );
            controller.setSetpoint(newSetpoint);
        }
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        rocketSubsystem.setThrottle(0.0);
       // rocketSubsystem.deactivatePartWithTag(AIRBRAKE_TAG);
    }
}