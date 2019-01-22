/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Superclass of all commands
 */
public abstract class CommandBase extends Command {
	/**
	 * {@inheritDoc}
	 */
	public CommandBase() {
		super();
	}
	
	/**
	 * {@inheritDoc}
	 */
	public CommandBase(double timeout) {
		super(timeout);
	}
	
	/**
	 * Calls {@link #end()} if {@link #cancel()} is called.
	 */
	@Override
	protected void interrupted() {
		this.end();
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected abstract void initialize();

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected abstract void execute();

	
	/**
	 * {@inheritDoc}
	 */
	@Override
	protected abstract boolean isFinished();

	
	/**
	 * {@inheritDoc}
	 */
	@Override
	protected abstract void end();
}