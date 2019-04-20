/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.Elevator;

public class GrabBall extends CommandGroup {
  public GrabBall(Elevator elevator) {
    addSequential(new ElevateGrab(elevator));
    addSequential(new RotateArm(elevator, -170));
    addSequential(new Elevate(elevator, 7.3));
  }
}
