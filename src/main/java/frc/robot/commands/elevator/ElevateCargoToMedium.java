/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Elevator;

/**
 * Sets the height of the elevator to place cargo medium
 */
public class ElevateCargoToMedium extends Elevate {
	private static final double CARGO_PORT_MEDIUM = 14.4;

	/**
	 * Sets the height of the elevator to place cargo medium
	 * @param elevator the elevator
	 */
	public ElevateCargoToMedium(Elevator elevator) {
		super(elevator, CARGO_PORT_MEDIUM);
  }
}
