/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Climber;

public class ClimbWithThrottle extends Command {
  private Joystick left, right;
  Climber climber;

  public ClimbWithThrottle(Joystick left, Joystick right, Climber climber) {
    this.left = left;
    this.right = right;
    this.climber = climber;
    requires(climber);  
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    System.out.println("LEFT: " + left.getZ() + " RIGHT: " + right.getZ());
    climber.set(left.getZ(), right.getZ(), 0);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    climber.stop();
  }
}
