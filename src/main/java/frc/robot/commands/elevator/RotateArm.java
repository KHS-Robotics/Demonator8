/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Elevator;

public class RotateArm extends Command {
  private Elevator elevator;
  private double setPoint;
  public RotateArm(Elevator elevator, double setPoint) {
    this.elevator = elevator;
    this.setPoint=setPoint;
    requires(elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    elevator.setArmRotation(setPoint);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return elevator.armOnTarget(setPoint);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }
}