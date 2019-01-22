/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.CargoIntake;

public class StopIntake extends InstantCommand {

  private CargoIntake intake;

  public StopIntake(CargoIntake intake) {
    super();
    this.intake = intake;
    this.requires(intake);
  }

  @Override
  protected void initialize() {
    intake.stop();
  }

}
