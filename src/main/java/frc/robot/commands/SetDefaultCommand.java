/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Subsystem;

public class SetDefaultCommand extends InstantCommand {
  private Subsystem subsystem;
  private Command command;

  public SetDefaultCommand(Subsystem subsystem, Command command) {
      this.subsystem = subsystem;
      this.command = command;
      
      this.requires(subsystem);
  }

  @Override
  protected void execute() {
      subsystem.setDefaultCommand(command);
  }
}