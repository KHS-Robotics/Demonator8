/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.tankdrive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.TankDrive;

/**
 * Add your docs here.
 */
public class DriveStraightJoystick extends InstantCommand {
  private double angle;

  private Joystick joystick;
  private TankDrive drive;
  /**
   * Add your docs here.
   */
  public DriveStraightJoystick(TankDrive drive, Joystick joystick) {
    super();
    this.requires(drive);
    this.drive = drive;
    this.joystick = joystick;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    angle = drive.getHeading();
  }
  @Override
  protected void execute() {
    drive.setHeading(angle, joystick.getY());
  }
  @Override
  protected boolean isFinished(){
    return false;
  }
  @Override
  protected void end() {
    drive.stop();
  }
}