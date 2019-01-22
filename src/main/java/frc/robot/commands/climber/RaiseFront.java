/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Climber;

public class RaiseFront extends Command {

private Climber climber;

  public RaiseFront(Climber climber) {
    this.requires(climber);
    this.climber = climber;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    climber.set(1,0,0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return climber.getFrontLS();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    climber.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
