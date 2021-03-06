package frc.robot.commands.tankdrive;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.TankDrive;
import frc.robot.vision.PixyCam;
import frc.robot.logging.Logger;
import io.github.pseudoresonance.pixy2api.Pixy2Line.Vector;

public class VisionAlignTargetPixy extends Command {
  private static final PowerDistributionPanel pdp = new PowerDistributionPanel();
  private static final int MOTOR1 = 0, MOTOR2 = 1, MOTOR3 = 14, MOTOR4 = 15;
  private static final double CURRENT_SPIKE = 12;

  private TankDrive drive;
  private PixyCam pixy;

  private boolean arrived, currentSpike;

  private int framesWithoutLine;
  private int loops;

  public VisionAlignTargetPixy(TankDrive drive, PixyCam pixy) {
    super(3);
    this.drive = drive;
    this.pixy = pixy;
    requires(drive);
  }

  @Override
  protected void initialize() {
    arrived = false;
    framesWithoutLine = 0;
    loops = 0;
  }

  @Override
  protected void execute() {
    Vector pixyLine = pixy.getLongestLine();
    
    if(pixyLine != null)
    {
      //pixy.getRealLine(pixyLine);

      framesWithoutLine = 0;
      double pixyX = pixyLine.getX0();

      if(pixyLine.getY0() > pixyLine.getY1())
      {
        pixyX = pixyLine.getX0();
      }
      else
      {
        pixyX = pixyLine.getX1();
      }

      pixyX -= 39;

      double lineSlope = ((double)(pixyLine.getX0() - pixyLine.getX1())) / (pixyLine.getY0() - pixyLine.getY1());
  
      //SmartDashboard.putNumber("Pixy Line Angle", pixy.getLineAngle());
      Logger.debug("Pixy Line Slope: " +  lineSlope);

      int slopeOffset = (int)(lineSlope * 10);

      Logger.debug("Pixy Slope Offset: " + slopeOffset);
      Logger.debug("Pixy Current X: " +  pixyX);

      //limit the amount of overcorrectoins
      if(slopeOffset > 20)
      {
        slopeOffset = 20;
      }
      else if(slopeOffset < -20)
      {
        slopeOffset = -20;
      }
      
      drive.setHeading(drive.getHeading() + (slopeOffset + pixyX) / 4, -0.50);
    }
    else
    {
      framesWithoutLine++;
    }
  
    double current = (pdp.getCurrent(MOTOR1) + pdp.getCurrent(MOTOR2) + pdp.getCurrent(MOTOR3) + pdp.getCurrent(MOTOR4)) / 4.0d;
    if(current > CURRENT_SPIKE && loops > 30) {
      arrived = true;
    }
    loops++;
  }

  @Override
  protected boolean isFinished() {
    return arrived || this.isTimedOut();
  }

  @Override
  protected void end() {
    drive.stop();
  }
}
