/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auton;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.tankdrive.DriveStraightDistance;
import frc.robot.subsystems.TankDrive;

public class AutoCloseHatch extends CommandGroup {

  public AutoCloseHatch(TankDrive drive) {
    addSequential(new DriveStraightDistance(drive, 0, 0, 0));
  }
}
