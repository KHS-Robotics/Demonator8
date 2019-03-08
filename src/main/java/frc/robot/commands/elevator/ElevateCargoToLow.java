/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import frc.robot.subsystems.Elevator;

/**
 * Sets the height of the elevator to place cargo low
 */
public class ElevateCargoToLow extends Elevate {
	private static final double CARGO_PORT_LOW = 4.8;

	/**
	 * Sets the height of the elevator to place cargo low
	 * @param elevator the elevator
	 */
	public ElevateCargoToLow(Elevator elevator) {
		super(elevator, CARGO_PORT_LOW);
	}
}