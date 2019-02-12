/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.tankdrive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.TankDrive;

public class DriveWithXbox extends Command {
  TankDrive drive;
  XboxController xbox;
  public DriveWithXbox(TankDrive drive, XboxController xbox) {
    this.drive = drive;
    this.xbox = xbox;
    this.requires(drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double forward = xbox.getTriggerAxis(Hand.kRight);
    double reverse = - xbox.getTriggerAxis(Hand.kLeft);
    double y = forward + reverse, x = xbox.getX(Hand.kLeft);

    drive.set(x + y, y - x);
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
