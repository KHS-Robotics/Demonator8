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
  private Spark fl, fr, rl, rr, dl, dr;
  private DigitalInput front, back;

  public Climber(Spark fl, Spark fr, Spark rl, Spark rr, Spark dl, Spark dr, DigitalInput front, DigitalInput back) {
    this.fl = fl;
    this.fr = fr;
    this.rl = rl;
    this.rr = rr;
    this.dl = dl;
    this.dr = dr;
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
    if(!(getFrontLS() && front < 0)) {
      fl.set(front);
      fr.set(front);
    }
    if(!(getBackLS() && back < 0)) {
      rl.set(back);
      rr.set(back);
    }
    dl.set(drive);
    dr.set(drive);
  }

  public boolean getFrontLS() {
    return front.get();
  }

  public boolean getBackLS() {
    return back.get();
  }
}
