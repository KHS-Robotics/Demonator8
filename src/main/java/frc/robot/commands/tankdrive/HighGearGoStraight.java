/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.tankdrive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.TankDrive;

public class HighGearGoStraight extends CommandGroup {
  public HighGearGoStraight(TankDrive drive, Joystick stick) {
    addSequential(new ShiftHigh(drive));
    addSequential(new DriveStraightJoystick(drive, stick));
  }
}
