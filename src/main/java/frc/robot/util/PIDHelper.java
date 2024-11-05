// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDHelper {
    private final String name;
    private double kP;
    private double kI;
    private double kD;
    private double setpoint;
    private double positionTolerance;
    private double velocityTolerance;

    /**
     * Creates a new PIDHelper with default values.
     * @param name The name prefix for SmartDashboard entries
     * @param defaultP Default P gain
     * @param defaultI Default I gain
     * @param defaultD Default D gain
     * @param defaultSetpoint Default setpoint
     * @param defaultPosTolerance Default position tolerance
     * @param defaultVelTolerance Default velocity tolerance
     */
    public PIDHelper(String name, double defaultP, double defaultI, double defaultD,
                    double defaultSetpoint, double defaultPosTolerance, double defaultVelTolerance) {
        this.name = name;
        
        // Initialize SmartDashboard entries with default values
        SmartDashboard.putNumber(name + "/kP", defaultP);
        SmartDashboard.putNumber(name + "/kI", defaultI);
        SmartDashboard.putNumber(name + "/kD", defaultD);
        SmartDashboard.putNumber(name + "/Setpoint", defaultSetpoint);
        SmartDashboard.putNumber(name + "/Position Tolerance", defaultPosTolerance);
        SmartDashboard.putNumber(name + "/Velocity Tolerance", defaultVelTolerance);
        
        // Initialize local variables
        updateValues();
    }

    /**
     * Updates all values from SmartDashboard.
     */
    public void updateValues() {
        kP = SmartDashboard.getNumber(name + "/kP", kP);
        kI = SmartDashboard.getNumber(name + "/kI", kI);
        kD = SmartDashboard.getNumber(name + "/kD", kD);
        setpoint = SmartDashboard.getNumber(name + "/Setpoint", setpoint);
        positionTolerance = SmartDashboard.getNumber(name + "/Position Tolerance", positionTolerance);
        velocityTolerance = SmartDashboard.getNumber(name + "/Velocity Tolerance", velocityTolerance);
    }

    /**
     * Publishes current error and output to SmartDashboard for monitoring.
     * @param currentError The current error
     * @param output The current PID output
     */
    public void publishTelemetry(double currentError, double output) {
        SmartDashboard.putNumber(name + "/Current Error", currentError);
        SmartDashboard.putNumber(name + "/PID Output", output);
    }

    // Getters
    public double getP() { return kP; }
    public double getI() { return kI; }
    public double getD() { return kD; }
    public double getSetpoint() { return setpoint; }
    public double getPositionTolerance() { return positionTolerance; }
    public double getVelocityTolerance() { return velocityTolerance; }
}