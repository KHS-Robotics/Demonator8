/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.logging;

/**
 * Severity for the Logger
 * 
 * @see org.usfirst.frc.team4342.robot.logging.Logger
 */
public enum Severity 
{
	ERROR(3), WARNING(4), INFO(6), DEBUG(7);
	
	private final int severity;
	
	/**
	 * Creates a new severity with the specified level 
	 * following syslog standard
	 * @param severity the severity from 0 to 7 (syslog standard)
	 */
	private Severity(int severity) 
	{
		this.severity = severity;
	}

	/**
	 * Gets the int value of the severity
	 * @return the int value of the severity following syslog standards
	 */
	public int level() 
	{
		return severity;
	}
}