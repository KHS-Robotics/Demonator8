/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.tankdrive;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.TankDrive;

public class ShiftLow extends InstantCommand
{
	private static final int DEBOUNCE_TIME_MS = 500;
	private static long lastTimeUsed;
	
	private TankDrive drive;
	
	public ShiftLow(TankDrive drive)
	{
		this.requires(drive);
		
		this.drive = drive;
	}
	
	@Override
	protected void initialize()
	{
		if(System.currentTimeMillis() - lastTimeUsed >= DEBOUNCE_TIME_MS)
			drive.shiftLow();
		
		lastTimeUsed = System.currentTimeMillis();
	}
}