// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ActivatePart;
import frc.robot.commands.AltitudeTarget;
import frc.robot.commands.LaunchCommand;
import frc.robot.commands.ThrottleCommand;
import frc.robot.commands.VelocityTarget;
import frc.robot.krpc.KRPCWrapper;
import frc.robot.krpc.VesselSubsystem;
import frc.robot.krpc.parts.DecouplerPart;
import frc.robot.krpc.parts.EnginePart;
import frc.robot.krpc.parts.FairingPart;
import frc.robot.krpc.parts.Part;
import frc.robot.krpc.parts.ProbeCorePart;
import frc.robot.subsystems.RocketSubsystem;
import frc.robot.util.KSPPartType;
import frc.robot.util.PIDHelper;
import krpc.client.services.SpaceCenter;

public class RobotContainer {
    private final KRPCWrapper krpc = new KRPCWrapper(false);
    private final CommandXboxController controller = new CommandXboxController(0);
    private final VesselSubsystem vesselSubsystem = new VesselSubsystem(krpc);
    
    // Get core parts
    private ProbeCorePart probeCore;
    private EnginePart firstStageEngine;
    private EnginePart secondStageEngine;
    private DecouplerPart stageDecoupler;
    private FairingPart payloadFairing;

    public RobotContainer() {

		//System.out.println("\n=== Printing all vessel parts ===");
		//vesselSubsystem.printVesselParts(true);
		//System.out.println("=== End vessel parts print ===\n");
		findAndInitializeParts();

      configureBindings();
    }

    private void findAndInitializeParts() {

		// Find probe core
		Part probeCoreParts = vesselSubsystem.getFirstPartByTag("BPROBE_main");
		if (probeCoreParts instanceof ProbeCorePart core) {
			probeCore = core;
			System.out.println("Found probe core: " + probeCore.getName());
		}
		
		// Find first stage engine
		Part engineParts = vesselSubsystem.getFirstPartByTag("ENGINE_first");
			if (engineParts instanceof EnginePart engine) {
				firstStageEngine = engine;
				System.out.println("Found first stage engine: " + engine.getName());
			}

		// Find second stage engine
		Part secondStageEngineParts = vesselSubsystem.getFirstPartByTag("ENGINE_second");
			if (secondStageEngineParts instanceof EnginePart engine) {
				secondStageEngine = engine;
				System.out.println("Found second stage engine: " + engine.getName());
			}
		
        // Find stage decoupler
        Part decouplerParts = vesselSubsystem.getFirstPartByTag("DECOUPLER_second_stage");
            if (decouplerParts instanceof DecouplerPart decoupler) {
                stageDecoupler = decoupler;
                System.out.println("Found stage decoupler: " + decoupler.getName());
            }
        
        // Find payload fairing
        Part fairingParts = vesselSubsystem.getFirstPartByTag("FAIRING_payload");
            if (fairingParts instanceof FairingPart fairing) {
                payloadFairing = fairing;
                System.out.println("Found payload fairing: " + fairing.getName());
            }
        
        // Verify all parts were found
        boolean allPartsFound = true;
        if (probeCore == null) {
            System.out.println("WARNING: Probe core not found!");
            allPartsFound = false;
        }
        if (firstStageEngine == null) {
            System.out.println("WARNING: First stage engine not found!");
            allPartsFound = false;
        }
        if (secondStageEngine == null) {
            System.out.println("WARNING: Second stage engine not found!");
            allPartsFound = false;
        }
        if (stageDecoupler == null) {
            System.out.println("WARNING: Stage decoupler not found!");
            allPartsFound = false;
        }
        if (payloadFairing == null) {
            System.out.println("WARNING: Payload fairing not found!");
            allPartsFound = false;
        }

        if (allPartsFound) {
            System.out.println("All parts found successfully!");
        }
    }

    private void configureBindings() {
        // Basic probe controls
        controller.rightBumper()
            .onTrue(probeCore.toggleSAS(true))
            .onFalse(probeCore.toggleSAS(false));

        // D-pad controls for orientation
        controller.povUp().onTrue(probeCore.holdPrograde());
        controller.povDown().onTrue(probeCore.holdRetrograde());
        controller.povRight().onTrue(probeCore.holdNormal());
        controller.povLeft().onTrue(probeCore.holdAntiNormal());

        // Engine controls
        controller.leftTrigger().whileTrue(Commands.run(() -> 
            firstStageEngine.setThrottle(controller.getLeftTriggerAxis())
        ));

        controller.rightTrigger().whileTrue(Commands.run(() -> 
            secondStageEngine.setThrottle(controller.getRightTriggerAxis())
        ));

        // Manual stage controls
        controller.x().onTrue(createStageSequence());
        controller.y().onTrue(Commands.runOnce(() -> payloadFairing.deploy()));
        
        // Automated launch sequence
        controller.start().onTrue(createLaunchSequence());

        // Emergency controls
        controller.back().onTrue(createAbortSequence());
    }

    private Command createStageSequence() {
        return Commands.sequence(
            Commands.print("Starting staging sequence..."),
            Commands.runOnce(() -> firstStageEngine.shutdown()),
            new WaitCommand(0.5),
            Commands.runOnce(() -> stageDecoupler.decouple()),
            new WaitCommand(0.5),
            Commands.runOnce(() -> secondStageEngine.activate()),
            Commands.print("Staging complete")
        );
    }

	private Command createLaunchSequence() {
		return Commands.sequence(
			// Pre-launch checks
			Commands.runOnce(() -> {
				System.out.println("Beginning launch sequence...");
				probeCore.setSAS(true);
				probeCore.setSASMode(ProbeCorePart.SASMode.STABILITY_ASSIST);
			}),
			
			// First stage
			Commands.runOnce(() -> {
				System.out.println("Igniting first stage...");
				firstStageEngine.activate();
				firstStageEngine.setThrottle(1.0);
			}),
			Commands.waitUntil(() -> krpc.getAltitude() > 10000),
			
			// Gravity turn
			Commands.runOnce(() -> {
				System.out.println("Beginning gravity turn...");
				probeCore.setSASMode(ProbeCorePart.SASMode.PROGRADE);
			}),
			Commands.waitUntil(() -> krpc.getAltitude() > 35000),
			
			// Stage separation
			Commands.runOnce(() -> {
				System.out.println("Preparing for stage separation...");
				firstStageEngine.setThrottle(0.0);
				firstStageEngine.shutdown();
			}),
			Commands.waitSeconds(1.0),
			Commands.runOnce(() -> stageDecoupler.decouple()),
			Commands.waitSeconds(1.0),
			
			// Second stage
			Commands.runOnce(() -> {
				System.out.println("Igniting second stage...");
				secondStageEngine.activate();
				secondStageEngine.setThrottle(1.0);
			}),
			Commands.waitUntil(() -> krpc.getAltitude() > 70000),
			
			// Coast phase
			Commands.runOnce(() -> {
				System.out.println("Entering coast phase...");
				secondStageEngine.setThrottle(0.0);
			}),
			Commands.waitUntil(() -> krpc.getAltitude() > 100000),
			
			// Deploy payload
			Commands.runOnce(() -> {
				System.out.println("Deploying payload...");
				payloadFairing.deploy();
			}),
			
			Commands.runOnce(() -> System.out.println("Launch sequence complete!"))
		);
	}

    private Command createAbortSequence() {
        return Commands.sequence(
            Commands.print("ABORT SEQUENCE INITIATED"),
            Commands.runOnce(() -> {
                firstStageEngine.shutdown();
                secondStageEngine.shutdown();
                probeCore.emergencyStabilize();
            }),
            Commands.runOnce(() -> payloadFairing.deploy()),
            Commands.print("Abort sequence complete")
        );
    }

    // Utility methods for telemetry
	private Command createTelemetryCommand() {
		return Commands.run(() -> {
			System.out.printf("Altitude: %.1f m\n", krpc.getAltitude());
			System.out.printf("Velocity: %.1f m/s\n", krpc.getOrbitalSpeed());
			System.out.printf("Stage 1 Throttle: %.2f\n", firstStageEngine.getThrottle());
			System.out.printf("Stage 2 Throttle: %.2f\n", secondStageEngine.getThrottle());
		}).repeatedly();
	}

    public Command getAutonomousCommand() {
        // Return the automated launch sequence for autonomous mode
        return createLaunchSequence();
    }
}