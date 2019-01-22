/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.CargoIntake;

/**
 * Add your docs here.
 */
public class StopIntake extends InstantCommand {

  private CargoIntake intake;
  /**
   * Add your docs here.
   */
  public StopIntake() {
    super();
    this.requires(intake);
    
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    intake.stop();
  }

}
