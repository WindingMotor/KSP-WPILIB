// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.krpc.KRPCWrapper;

public class RocketSubsystem extends SubsystemBase {
    private final KRPCWrapper krpc;
    private boolean isLaunched = false;

    /**
     * Creates a new RocketSubsystem.
     */
    public RocketSubsystem() {
        // Initialize KRPCWrapper with debugging enabled
        krpc = new KRPCWrapper(false);
    }

    @Override
    public void periodic() {
        // Update SmartDashboard with telemetry data if streams are available
        if (krpc != null) {
            SmartDashboard.putNumber("AltitudeM", krpc.getAltitude());
            SmartDashboard.putNumber("Surface Speed", krpc.getSurfaceSpeed());
            SmartDashboard.putNumber("Orbital Speed", krpc.getOrbitalSpeed());
            SmartDashboard.putNumber("Throttle", krpc.getThrottle());
            SmartDashboard.putBoolean("Launched", isLaunched);

            // Velocity vector
            Translation3d velocityVector = krpc.getVelocityVector();
            SmartDashboard.putNumber("Velocity X", velocityVector.getX());
            SmartDashboard.putNumber("Velocity Y", velocityVector.getY());
            SmartDashboard.putNumber("Velocity Z", velocityVector.getZ());

        } else {
            SmartDashboard.putString("KRPC Status", "Unavailable");
        }
    }

    /**
     * Launches the rocket by setting throttle to maximum and activating the first stage.
     */
    public void launch() {
        if (krpc != null) {
            krpc.setThrottle(1.0);
            krpc.activateNextStage();
            isLaunched = true;
        }
    }

    /**
     * Sets the throttle to a specific value.
     * @param throttle Value between 0.0 and 1.0
     */
    public void setThrottle(double throttle) {
        if (krpc != null) {
            krpc.setThrottle(throttle);
        }
    }

    /**
     * Activates the next stage of the rocket.
     */
    public void stage() {
        if (krpc != null) {
            krpc.activateNextStage();
        }
    }

    /**
     * Gets the current altitude in meters.
     * @return Current altitude in meters
     */
    public double getAltitude() {
        if (krpc != null) {
            return krpc.getAltitude();
        } else {
            return 0.0;
        }
    }


    /**
     * Gets the current surface speed in meters per second.
     * @return Current surface speed in meters per second
     */
    public double getSurfaceSpeed() {
        if (krpc != null) {
            return krpc.getSurfaceSpeed();
        } else {
            return 0.0;
        }
    }

    /**
     * Gets the current orbital speed in meters per second.
     * @return Current orbital speed in meters per second
     */
    public double getOrbitalSpeed() {
        if (krpc != null) {
            return krpc.getOrbitalSpeed();
        } else {
            return 0.0;
        }
    }

    public double getZVelocity() {
        if (krpc != null) {
            return -krpc.getVelocityVector().getZ();
        } else {
            return 0.0;
        }
    }

    /**
     * Gets the current throttle setting.
     * @return Current throttle value between 0.0 and 1.0
     */
    public double getThrottle() {
        if (krpc != null) {
            return krpc.getThrottle();
        } else {
            return 0.0;
        }
    }

    /**
     * Activates the part with the given tag.
     * @param tag Tag of the part to activate
     */
    public void activatePartWithTag(String tag) {
        if (krpc != null) {
            krpc.activatePartWithTag(tag);
        }
    }

    /**
     * Deactivates the part with the given tag.
     * @param tag Tag of the part to deactivate
     */
    public void deactivatePartWithTag(String tag) {
        if (krpc != null) {
            krpc.deactivatePartWithTag(tag);
        }
    }

}