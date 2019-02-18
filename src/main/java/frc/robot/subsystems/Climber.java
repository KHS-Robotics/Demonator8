/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;

/**
 * Add your docs here.
 */
public class Climber extends SubsystemBase {
  private Spark fClimb, bClimb, driveR, driveL;
  private DigitalInput front, back;

  public Climber(Spark fClimb, Spark bClimb, Spark driveL, Spark driveR, DigitalInput front, DigitalInput back) {
    this.fClimb = fClimb;
    this.bClimb = bClimb;
    this.driveL = driveL;
    this.driveR = driveR;
    this.front = front;
    this.back = back;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void stop() {
    set(0, 0, 0);
  }

  public void set(double front, double back, double drive) {
    /*if(getFrontLS() && front > 0)
    {
      front = 0;
    }
    if(getBackLS() && back > 0)
    {
      back = 0;
    }*/
    
    fClimb.set(-front);
    bClimb.set(-back);
    driveR.set(-drive);
    driveL.set(drive);
  }

  public boolean getFrontLS() {
    return front.get();
  }

  public boolean getBackLS() {
    return back.get();
  }
}
