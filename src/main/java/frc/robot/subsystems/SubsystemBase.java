/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * Superclass of all subsystems
 */
public abstract class SubsystemBase extends Subsystem {
	/**
     * Stops all motors associated with the subsystem
     */
	public abstract void stop();

	/**
	 * Sets the default command to <code>null</code>
	 */
	@Override
	protected void initDefaultCommand() {
		this.setDefaultCommand(null);
	}

	/**
	 * <p>Builds values to send to the <code>SmartDashboard</code></p>
	 */
	@Override
	public void initSendable(SendableBuilder builder) {
		
	}
}