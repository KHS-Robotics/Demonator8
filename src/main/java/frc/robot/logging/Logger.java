/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.logging;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Logger to make it easier to log the Driver Station and stdout/stderr
 */
public class Logger 
{
	private Logger() {}
	
	private static Severity severity = Severity.DEBUG;
	
	/**
	 * Sets the minimum severity to log
	 * @param sev the minimum severity to log
	 */
	public static void setSeverity(Severity sev)
	{
		severity = sev;
	}
	
	/**
	 * Logs a message to the Driver Station
	 * @param sev the severity of the message
	 * @param message the message to send
	 * @param t the exception to log to stderr
	 */
	public static void log(Severity sev, String message, Throwable t)
	{
		if(sev.level() <= severity.level())
		{
			if (sev.level() <= Severity.ERROR.level())
				DriverStation.reportError(message, false);
			else if(sev.level() <= Severity.WARNING.level())
				DriverStation.reportWarning(message, false);
			else
				System.out.println(sev.toString().toUpperCase() + ": " + message);
				
			if(t != null)
			{
				String stacktrace = "\n";
				for(StackTraceElement ste : t.getStackTrace())
				{
					stacktrace += ste.toString() + "\n";
				}
				
				System.err.println(stacktrace);
			}
		}
	}
	
	/**
	 * Debug message to send to the Driver Station
	 * @param message the message to report to the Driver Station
	 */
	public static void debug(String message)
	{
		log(Severity.DEBUG, message, null);
	}
	
	/**
	 * Info message to send to the Driver Station
	 * @param message the message to report to the Driver Station
	 */
	public static void info(String message)
	{
		log(Severity.INFO, message, null);
	}
	
	/**
	 * Warning message to send to the Driver Station
	 * @param message the message to report to the Driver Station
	 */
	public static void warning(String message)
	{
		log(Severity.WARNING, message, null);
	}
	
	/**
	 * Error message to send to the Driver Station
	 * @param message the message to report to the Driver Station
	 * @param t the exception to print to stderr
	 */
	public static void error(String message, Throwable t)
	{
		log(Severity.ERROR, message, t);
	}
}