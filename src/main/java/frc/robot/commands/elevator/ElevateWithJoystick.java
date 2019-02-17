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
	private static final double DEADBAND = 0.03;
	
	private boolean idle, initializedIdle;
	
	private Joystick joystick;
	private Elevator elevator;
	
	/**
	 * Command to control the elevator with a joystick
	 * @param joystick the joystick
	 * @param elevator the elevator
	 */
	public ElevateWithJoystick(Elevator elevator, Joystick joystick) {
		this.joystick = joystick;
		this.elevator = elevator;
		
		this.requires(elevator);
	}

	@Override
	protected void initialize() {
		if(!elevator.getLS()) {
			elevator.setSetpoint(elevator.getPosition());
			elevator.enable();
		}
	}

	@Override
	protected void execute() {
		final double INPUT = joystick.getRawAxis(2);
		double output = INPUT;

		if(elevator.getLS() && INPUT < 0) {
			output = 0;
		}
		// if(elevator.getElevatorHeight() >= MAX_ELEVATOR_HEIGHT)

		elevator.set(output);
	}

	@Override
	protected void end() {
		elevator.stop();
	}
	
	private static boolean checkJoystickDeadband(double a) {
		return Math.abs(a) < DEADBAND;
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
}