/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import frc.robot.OI;
import frc.robot.commands.DriveWithJoysticks;
/**
 * Add your docs here.
 */
public class TankDrive extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
private Spark left, right;

public TankDrive (Spark left, Spark right)
{
  this.left = left;
  this.right = right;
}

public void setDrive (double l, double r)
{
  left.set(l);
  right.set(r);
}

  @Override
  public void initDefaultCommand() {
    OI oi = OI.getInstance();
		this.setDefaultCommand(new DriveWithJoysticks(oi.drive, oi.leftJoystick, oi.rightJoystick));
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
