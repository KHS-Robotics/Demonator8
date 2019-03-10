/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.subsystems.Climber;

public class ClimbPID extends Command {
  private Climber climber;

  public ClimbPID(Climber climber) {
    this.climber = climber;
    requires(climber);
  }

  @Override
  protected void initialize() {
    climber.autoClimb();
  }

  @Override
  protected void execute() {}

  @Override
  protected boolean isFinished() {
    return climber.getLS();
  }

  @Override
  protected void end() {
    climber.stop();
  }
}
