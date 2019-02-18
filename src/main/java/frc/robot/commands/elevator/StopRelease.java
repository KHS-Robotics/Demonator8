/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Elevator;

public class StopRelease extends InstantCommand {
  private Elevator elevator;

  public StopRelease(Elevator elevator) {
    super();
    this.elevator = elevator;
    this.requires(elevator);
  }

  @Override
  protected void initialize() {
    elevator.setIntake(0, 0);
  }

}
