/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Climber;

public class StartClimbDrive extends InstantCommand {
  private Climber climber;
  private double power;
  public StartClimbDrive(Climber climber, double power) {
    super();
    this.climber = climber;
    this.power = power;
    this.requires(climber);
  }
 
  @Override
  protected void initialize() {
    climber.set(0, 0, power);
  }

  @Override
  protected void end() {
  }
}
