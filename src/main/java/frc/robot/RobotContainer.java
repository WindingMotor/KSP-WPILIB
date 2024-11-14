// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ActivatePart;
import frc.robot.commands.AltitudeTarget;
import frc.robot.commands.LaunchCommand;
import frc.robot.commands.ThrottleCommand;
import frc.robot.commands.VelocityTarget;
import frc.robot.subsystems.RocketSubsystem;
import frc.robot.util.KSPPartType;
import frc.robot.util.PIDHelper;

public class RobotContainer {

    private final RocketSubsystem rocketSubsystem = new RocketSubsystem();
	private final CommandXboxController controller = new CommandXboxController(0);



	private final PIDHelper velocityPIDHelper = new PIDHelper(
		"Velocity Control",  
		0.05,               // P gain
		0.001,              // I gain
		0.01,               // D gain
		10.0,               // default setpoint
		1.0,                // position tolerance
		0.1                 // velocity tolerance
	);


    public RobotContainer() {
        configureBindings();

        // Create a setpoint supplier that scales axis 5 
		/* 
        rocketSubsystem.setDefaultCommand(
            new VelocityTarget(
                rocketSubsystem,
                () -> (controller.getRawAxis(5) + 1.0) * 65.0, // Scale from -1,1 to 0,~130
                velocityPIDHelper
            )
        );
		*/
    }


	private void configureBindings() {
		
		//controller.a().onTrue(new LaunchCommand(rocketSubsystem, 500)); // Target 500m altitude
		controller.b().onTrue(new ActivatePart(rocketSubsystem, KSPPartType.FAIRING, "payload-fairing", true));

		/* 
		controller.a().onTrue(
			new Alti(rocketSubsystem,
			() -> controller.getRawAxis(5),
			() -> controller.b().getAsBoolean())
		);
		*/
		

	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
