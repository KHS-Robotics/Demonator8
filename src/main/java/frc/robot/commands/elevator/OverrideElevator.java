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
import frc.robot.subsystems.Elevator;

public class OverrideElevator extends Command {

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
    elevator.setOverride(true);
    elevator.stop();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    elevator.set(stick.getRawAxis(ButtonMap.SwitchBox.ELEVATOR_AXIS));
    elevator.setArm(stick.getRawAxis(ButtonMap.SwitchBox.ARM_AXIS));
    elevator.setIntake(stick.getRawAxis(ButtonMap.SwitchBox.ELEVATOR_INTAKE_AXIS), stick.getRawAxis(ButtonMap.SwitchBox.ELEVATOR_INTAKE_AXIS));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    elevator.setOverride(false);
    elevator.stop();
  }
}
