/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.ButtonMap;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

/**
 * Command to control the elevator with a joystick
 */
public class ElevateWithJoystick extends Command {
	public static final double DEADBAND = 0.075;
	private boolean initializedIdleElev, isIdleElev, initializedIdleArm, isIdleArm, initDisabled;

	private Joystick stick;
	private Elevator elevator;

	/**
	 * Command to control the elevator with a joystick
	 * 
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
		initializedIdleElev = false;
		initializedIdleArm = false;
	}

	@Override
	protected void execute() {
		if (elevator.getLS()) {
			elevator.reset();
			elevator.disable();
		}

		if (Robot.enabled()) {
			double inputElev = -stick.getRawAxis(ButtonMap.SwitchBox.ELEVATOR_AXIS);
			double inputArm = stick.getRawAxis(ButtonMap.SwitchBox.ARM_AXIS);
			isIdleElev = Math.abs(inputElev) < DEADBAND;
			isIdleArm = Math.abs(inputArm) < DEADBAND;

			if (isIdleElev && !initializedIdleElev) {
				elevator.setSetpoint(elevator.getElevatorHeight());
				initializedIdleElev = true;
			}

			if (isIdleArm && !initializedIdleArm) {
				elevator.setArmRotation(elevator.getArmRotation());
				initializedIdleArm = true;
			}

			if(!isIdleElev) {
				elevator.disable();
				elevator.set(inputElev);
				initializedIdleElev = false;
			}

			if(!isIdleArm) {
				elevator.setArm(inputArm);
				initializedIdleArm = false;
			}

			elevator.setIntake(deadband(stick.getRawAxis(ButtonMap.SwitchBox.ELEVATOR_INTAKE_AXIS)), deadband(stick.getRawAxis(ButtonMap.SwitchBox.ELEVATOR_INTAKE_AXIS)));

			initDisabled = false;
		}

		if (!Robot.enabled() && !initDisabled) {
			elevator.stop();
			initializedIdleElev = false;
			initializedIdleArm = false;
			initDisabled = true;
		}
	}

	@Override
	protected void end() {
		//elevator.stop();
	}

	public double deadband(double input) {
		return Math.abs(input) > DEADBAND ? input : 0;
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
}