/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.elevator.StartGrab;
import frc.robot.commands.intake.StartIntake;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Elevator;

public class BothIntake extends CommandGroup {
  public BothIntake(Elevator elevator, CargoIntake intake) {
    addParallel(new StartGrab(elevator));
    addSequential(new StartIntake(intake));
  }
}
