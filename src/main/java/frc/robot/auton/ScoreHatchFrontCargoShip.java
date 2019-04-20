/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auton;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.elevator.RotateArm;
import frc.robot.commands.tankdrive.DriveStraightDistance;
import frc.robot.commands.tankdrive.VisionAlignTarget;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.TankDrive;
import frc.robot.vision.MoePiClient;
import frc.robot.vision.PixyCam;

public class ScoreHatchFrontCargoShip extends CommandGroup {
  public static final double DISTANCE_UNTIL_VISION = 72.0;
  
  public ScoreHatchFrontCargoShip(TankDrive drive, MoePiClient moepi, PixyCam pixy, Elevator elevator) {
    addParallel(new RotateArm(elevator, 90));
    addSequential(new DriveStraightDistance(drive, DISTANCE_UNTIL_VISION, 0, -0.8));
    addSequential(new VisionAlignTarget(drive, moepi, pixy));
  }
}
