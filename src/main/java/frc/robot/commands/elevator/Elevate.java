/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Elevator;

public class Elevate extends Command {

  protected final Elevator elevator;
	private double height;

	/**
	 * Elevate Command to set the elevator to a specified height
	 * @param elevator the elevator
	 * @param height the desired height of the elevator
	 */
	public Elevate(Elevator elevator, double height) {
		super(2.5);
		
		this.elevator = elevator;
		this.height = height;
		
		this.requires(elevator);
	}
	
	@Override
	protected void initialize() {
		elevator.setSetpoint(height);
		elevator.enable();
	}

	@Override
	protected boolean isFinished() {
		return elevator.onTarget() || this.isTimedOut();
  }

  @Override
	protected void execute() {}

	@Override
	protected void end() {}
}
