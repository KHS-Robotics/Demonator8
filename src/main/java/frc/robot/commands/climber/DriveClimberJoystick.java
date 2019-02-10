/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Climber;

public class DriveClimberJoystick extends InstantCommand {
  private Climber climber;
  private Joystick stick;
  public DriveClimberJoystick(Climber climber, Joystick stick) {
    super();
    this.climber = climber;
    this.stick = stick;
    this.requires(climber);
  }
 
  @Override
  protected void execute() {
    climber.set(0, 0, stick.getY());
  }

  @Override
  protected void end() {
    climber.stop();
  }

}
