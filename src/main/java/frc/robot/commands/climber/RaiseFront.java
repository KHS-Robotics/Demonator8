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
private int loop = 30;

  public RaiseFront(Climber climber) {
    this.requires(climber);
    this.climber = climber;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    climber.setPinions(-0.8, 0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // if(loop > 30) {
    //   climber.setPinions(-0.8,0);
    // } else {
    //   climber.setPinions(-1,0);
    // }

    // loop++;
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
