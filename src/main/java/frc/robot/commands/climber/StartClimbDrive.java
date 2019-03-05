/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Climber;

public class StartClimbDrive extends Command {
  private Climber climber;
  private double drivePower, holdingPower;
  public StartClimbDrive(Climber climber, double drivePower, double holdingPower) {
    super();
    this.climber = climber;
    this.drivePower = drivePower;
    this.holdingPower = holdingPower;
    this.requires(climber);
  }
 
  @Override
  protected void initialize() {
    climber.setPinions(holdingPower, 0);
    climber.setDrive(drivePower);
  }

  @Override
  protected void end() {
    climber.stop();
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}
