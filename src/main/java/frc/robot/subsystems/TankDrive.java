/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

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
private AHRS navx;

public TankDrive (Spark left, Spark right, AHRS navx)
{
  this.left = left;
  this.right = right;
  this.navx = navx;
}

public void setDrive (double l, double r)
{
  left.set(normalizeOutput(l));
  right.set(normalizeOutput(r));
}

/**
	 * Internal function to normalize a motor output
	 * @param output the unnormalized output
	 * @return the normalized output ranging from -1.0 to 1.0
	 */
	private static double normalizeOutput(double output)
	{
		if(output > 1)
			return 1.0;
		else if(output < -1)
			return -1.0;
		return output;
  }
  
  public void stopMotors()
  {
    this.setDrive(0,0);
  }

  public void resetNavx()
  {
    navx.reset();
  }

  public double getHeading()
  {
    return normalizeYaw(navx.getYaw());
  }

  private double normalizeYaw(double yaw)
  {
    while (yaw >= 180 )
    {
      yaw -= 360;
    }
    while (yaw <= 180)
    {
      yaw += 360;
    }
    return yaw;
  }

  @Override
  public void initDefaultCommand() {
    OI oi = OI.getInstance();
		this.setDefaultCommand(new DriveWithJoysticks(oi.drive, oi.leftJoystick, oi.rightJoystick));
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
