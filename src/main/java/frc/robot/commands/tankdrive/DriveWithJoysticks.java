/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.tankdrive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.TankDrive;

public class DriveWithJoysticks extends Command {
  private static final double SENSITIVITY = 0.25; // [0, 1]; 0 for linear, 1 for cubic
  private TankDrive drive;
  private Joystick left, right;
  
  public DriveWithJoysticks(TankDrive drive, Joystick left, Joystick right) {
    this.drive = drive;
    this.left = left;
    this.right = right;
    this.requires(drive);
  }
  
  @Override
  protected void execute() {
    drive.set(sensitivity(right.getY()), sensitivity(left.getY()));
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void initialize() {}

  @Override
  protected void end() {
    drive.stop();
  }

  private static double sensitivity(double input) {
    return SENSITIVITY * Math.pow(input, 3) + (1 - SENSITIVITY) * input;
  }
}