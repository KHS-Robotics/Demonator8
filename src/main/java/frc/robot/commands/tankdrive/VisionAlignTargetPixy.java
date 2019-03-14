package frc.robot.commands.tankdrive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.TankDrive;
import frc.robot.vision.PixyCam;
import io.github.pseudoresonance.pixy2api.Pixy2Line.Vector;

public class VisionAlignTargetPixy extends Command {
  private TankDrive drive;
  private PixyCam pixy;

  private boolean arrived;

  private int framesWithoutLine;

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
  }

  @Override
  protected void execute() {
    Vector pixyLine = pixy.getLongestLine();
    
    if(pixyLine != null)
    {
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

  @Override
  protected boolean isFinished() {
    return arrived || this.isTimedOut();
  }

  @Override
  protected void end() {
    drive.stop();
  }
}
