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
import frc.robot.vision.MoePiClient;
import frc.robot.vision.PixyCam;
import io.github.pseudoresonance.pixy2api.Pixy2Line.Vector;

public class XOffsetVision extends Command {
  private TankDrive drive;
  private MoePiClient udp;
  private PixyCam pixy;

  private final double cameraX = 14.5;
  private final double ultraSonicY = 15;

  private double offset = 0;
  private boolean firstTime = true;

  private boolean side; //right is true
  private boolean arrived;

  private int framesWithoutLine;

  public XOffsetVision(TankDrive drive, MoePiClient udp, PixyCam pixy) {
    this.drive = drive;
    this.udp = udp;
    this.pixy = pixy;
    requires(drive);
  }

  @Override
  protected void initialize() {
    // firstTime = true;
    side = (drive.getHeading() < 0);
    arrived = false;
    framesWithoutLine = 0;
    System.out.println(side);
  }

  public double cubicCorrection(double x) {
    // return 0.00266666667*Math.pow(x, 3) - 2.0666667*x;
    return 0.00133333*Math.pow(x, 3) - 1.033333 * x;
    // return 0.00333333333*Math.pow(x, 3) - 2.833333333*x;
  }

  private double calculateXOffset()
  {
    double alpha = Math.toRadians(-udp.getAngle());
    double theta = -Math.toRadians(drive.getHeading() - offset);
    double distance = drive.getAverageUltrasonic();

    double temp = cameraX - (ultraSonicY * Math.tan(alpha));

    double numerator = (Math.sin((Math.PI / 2) - alpha)) * temp;
    double a = Math.asin(numerator / distance);
    double c = (distance * Math.sin(-a + theta + alpha))/(Math.sin(Math.PI - theta));

    return Math.sin(theta) * (ultraSonicY + c);
  }

  private void DriveWithPixy()
  {
    Vector pixyLine = pixy.getLongestLine();
    
    if(pixyLine != null)
    {
      framesWithoutLine = 0;
      float pixyX = pixyLine.getX0();

      if(pixyLine.getY0() > pixyLine.getY1())
      {
        pixyX = pixyLine.getX0();
      }
      else
      {
        pixyX = pixyLine.getX1();
      }

      pixyX -= 39;
  
      SmartDashboard.putNumber("Pixy X", pixyX);
  
      drive.setHeading(drive.getHeading() + pixyX / 4, -0.5);
    }
    else
    {
      framesWithoutLine++;
    }

    if(framesWithoutLine > 3)
    {
      arrived = true;
    }
  }

  private void DriveWithMoePi()
  {
    if(udp.getAngle() != 0){//drive.getAverageUltrasonic() > 15) {
      

      // System.out.println("X offest Distance: " + x + "\nAlpha: " + Math.toDegrees(alpha) + "\nTheta: " + Math.toDegrees(theta));

      if(side == true) //approach from right
      {
        drive.setHeading(udp.getAngle() + 10, -0.4);
      }
      else
      {
        drive.setHeading(udp.getAngle() + 17, -0.4);
      }
      // SmartDashboard.putNumber("Drive-Correction", correction);
      // SmartDashboard.putNumber("X offset", x);
    }
  }

  @Override
  protected void execute() {

    // Vector line = pixy.getLongestLine();
    // if(line != null)
    // {
    //   pixy.getRealLine(line);
    // }
    
    // if(drive.getAverageUltrasonic() > 20)
    // {
    //   DriveWithMoePi();
    // }
    // else
    // {
      DriveWithPixy();
    // }
  }

  @Override
  protected boolean isFinished() {
    return arrived;
  }

  @Override
  protected void end() {
    drive.stop();
  }
}
