/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.tankdrive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.TankDrive;

public class DriveStraightDistance extends Command {
  private TankDrive drive;
  private double distance, angle, power, initialLeftDist, initialRightDist;


  public DriveStraightDistance(TankDrive drive, double distance, double angle, double power) {
    super(3);
    this.drive = drive;
    this.distance = distance;
    this.angle = angle;
    this.power = power;
    requires(drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    initialRightDist = drive.getRightDistance();
    initialLeftDist = drive.getLeftDistance();

    drive.setHeading(angle, power);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double r = drive.remainingDistance(distance, initialLeftDist, initialRightDist);
    SmartDashboard.putNumber("Drive-AutoDist", r);
    return r <= 0 || this.isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    drive.stop();
  }

}
