/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.tankdrive;

import edu.wpi.first.wpilibj.command.Command;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.TankDrive;
import frc.robot.vision.MoePiClient;;

public class DriveStraightAtTargetJoystick extends Command {
  private static final double JOYSTICK_DEADBAND = 0.03;
  private TankDrive drive;
  private MoePiClient vision;
  private Joystick joystick;

  public DriveStraightAtTargetJoystick(TankDrive drive, MoePiClient vision, Joystick joystick) {
    this.requires(drive);
    this.drive = drive;
    this.vision = vision;
    this.joystick = joystick;
  }

  @Override
  protected void initialize() {
    drive.lightOn();
  }

  @Override
  protected void execute() {
    double y = -joystick.getY();
    if(Math.abs(y) < JOYSTICK_DEADBAND) {
      y = 0;
    }

    drive.setHeading(vision.getLastRobotHeading() + vision.getAngle(MoePiClient.CAMERA_ANGLE_OFFSET), y);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    drive.stop();
    drive.lightOff();
  }
}