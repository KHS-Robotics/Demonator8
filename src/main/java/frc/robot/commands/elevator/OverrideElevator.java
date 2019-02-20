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

public class OverrideElevator extends Command {
  public static final double DEADBAND = 0.075;
	private boolean initializedIdle, isIdle;

  private Joystick stick;
  private Elevator elevator;

  public OverrideElevator(Joystick stick, Elevator elevator) {
    this.stick = stick;
    this.elevator = elevator;
    requires(elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
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

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    elevator.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    elevator.stop();
  }

  public double deadband(double input) {
    return Math.abs(input) > DEADBAND ? input : 0;
  }
}
