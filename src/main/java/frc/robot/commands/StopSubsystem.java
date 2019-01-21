/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.SubsystemBase;

public class StopSubsystem extends InstantCommand {
    SubsystemBase subsystem;

  public StopSubsystem(SubsystemBase subsystem) {
    super();
    this.subsystem = subsystem;
    requires(subsystem);
  }

  @Override
  protected void initialize() {
    subsystem.stop();
  }

}
