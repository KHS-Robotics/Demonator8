/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import frc.robot.subsystems.Elevator;

/**
 * Sets the height of the elevator relative to its current setpoint
 */
public class ElevateRelative extends Elevate {
	private double delta;

    /**
	 * Sets the height of the elevator relative to its current setpoint
	 * @param elevator the elevator
	 * @param delta the desired change in setpoint
	 */
	public ElevateRelative(Elevator elevator, double delta) {
		super(elevator, 0.0);

		this.delta = delta;
	}

	@Override
	protected void initialize() {
		elevator.setSetpointRelative(delta);
		elevator.enable();
	}

	@Override
	protected boolean isFinished() {
		return super.isFinished() || elevator.getLS();
	}
}
