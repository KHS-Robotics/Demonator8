/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Superclass for subsystems used on the robot for methods
 * that they may share
 */
public abstract class SubsystemBase extends Subsystem 
{
	/**
	 * Creates a new subsystem
	 */
	public SubsystemBase()
	{
		super();
	}
	
	/**
	 * {@inheritDoc}
	 */
	@Override
	protected void initDefaultCommand() 
	{
		this.setDefaultCommand(null);
	}
}