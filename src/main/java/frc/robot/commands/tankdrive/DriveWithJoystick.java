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

public class DriveWithJoystick extends Command {
  private static final double DEADBAND = 0.08;

  private Joystick joystick;
  private TankDrive drive;

  public DriveWithJoystick(TankDrive drive, Joystick joystick) {
    this.requires(drive);
    this.joystick = joystick;
    this.drive = drive;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double x = joystick.getX();
    double y = -joystick.getY();
    if(Math.abs(x) < DEADBAND) {
      x = 0;
    }
    if(Math.abs(y) < DEADBAND) {
      y = 0;
    }

    double left = y + x;
    double right = y - x;

    drive.set(left, right);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    drive.stop();
  }
}