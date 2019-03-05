/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Elevator;

/**
 * Command to control the elevator with a joystick
 */
public class ElevateWithJoystick extends Command {
	public static final double DEADBAND = 0.075;
	private boolean initializedIdle, isIdle;

	private Joystick stick;
	private Elevator elevator;
	
	/**
	 * Command to control the elevator with a joystick
	 * @param joystick the joystick
	 * @param elevator the elevator
	 */
	public ElevateWithJoystick(Elevator elevator, Joystick stick) {
		this.stick = stick;
		this.elevator = elevator;
		
		this.requires(elevator);
	}

	@Override
	protected void initialize() {
		initializedIdle = false;
	}

	@Override
	protected void execute() {
		
	double input = stick.getRawAxis(0);
    isIdle = Math.abs(input) <= DEADBAND;

    if(isIdle && !initializedIdle) {
      elevator.setArmRotation(elevator.getArmRotation());
      initializedIdle = true;
    }

    if(Math.abs(input) > DEADBAND) {
      initializedIdle = false;
    }

    elevator.set(deadband(-stick.getRawAxis(2)));

    if(!isIdle) {
      elevator.setArm(deadband(stick.getRawAxis(0)));
    }

    elevator.setIntake(deadband(stick.getRawAxis(1)), deadband(stick.getRawAxis(1)));
	}

	@Override
	protected void end() {
		elevator.stop();
	}
	
	public double deadband(double input) {
		return Math.abs(input) > DEADBAND ? input : 0;
	  }

	@Override
	protected boolean isFinished() {
		return false;
	}
}