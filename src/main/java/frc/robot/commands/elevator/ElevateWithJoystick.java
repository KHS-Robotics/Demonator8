/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;



import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.TeleopCommand;
import frc.robot.subsystems.Elevator;

/**
 * Command to control the elevator with a joystick
 */
public class ElevateWithJoystick extends TeleopCommand {
	private static final double DEADBAND = 0.03;
	
	private boolean idle, initializedIdle;
	
	private Joystick joystick;
	private JoystickButton override;
	private Elevator elevator;
	
	/**
	 * Command to control the elevator with a joystick
	 * @param joystick the joystick
	 * @param elevator the elevator
	 */
	public ElevateWithJoystick(Joystick joystick, JoystickButton override, Elevator elevator) {
		this.joystick = joystick;
		this.override = override;
		this.elevator = elevator;
		
		this.requires(elevator);
	}

	@Override
	protected void initialize() {
		if(!elevator.isAtBottom()) {
			elevator.setSetpoint(elevator.getPosition());
			elevator.enable();
		}
	}

	@Override
	protected void execute() {
		final double INPUT = joystick.getTwist();
		final boolean OVERRIDE = !override.get(); // switch is opposite
		elevator.setOverride(OVERRIDE);
		idle = checkJoystickDeadband(INPUT);

		// Emergency override in case sensors malfunction
		if(!idle && OVERRIDE) {
			elevator.disable();

			double input = INPUT > 0 ? INPUT : INPUT * 0.4;
			elevator.set(input);
			return;
		}
		else if(OVERRIDE) {
			elevator.stop();
			return;
		}
		
		if(elevator.isAtBottom()) {
			elevator.stop();
			elevator.reset();
			
			if(!idle && INPUT < 0) {
				return;
			}
		}

		if(!elevator.isAtBottom() && idle && !initializedIdle) {
			elevator.setSetpoint(elevator.getPosition()); // hold current height
			elevator.enable();
			initializedIdle = true;
		} 
		else if(!idle) {
			double input = 0.0;

			if(INPUT > 0 && elevator.getPosition() < 3600)
				input = INPUT;
			else if(INPUT < 0)
				input = INPUT * 0.6;
			
			elevator.disable();
			elevator.set(input);
			initializedIdle = false;
		}
	}

	@Override
	protected void end() {
		elevator.stop();
	}
	
	private static boolean checkJoystickDeadband(double a) {
		return Math.abs(a) < DEADBAND;
	}
}