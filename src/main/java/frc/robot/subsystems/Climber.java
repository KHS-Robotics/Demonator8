/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PIDController;

import com.kauailabs.navx.frc.AHRS;

/**
 * Add your docs here.
 */
public class Climber extends SubsystemBase implements PIDSource, PIDOutput {
  private Spark fClimb, bClimb, driveR, driveL;
  private DigitalInput front, back;

  private double pClimber, iClimber, dClimber;
  public static final double climbP = 0.04, climbI = 0.0, climbD = 0.025;
  private PIDController pitchPID;
  private PIDSourceType pidSourceType;

  private AHRS navx;

  public Climber(Spark fClimb, Spark bClimb, Spark driveL, Spark driveR, AHRS navx) {
    this.fClimb = fClimb;
    this.bClimb = bClimb;
    this.driveL = driveL;
    this.driveR = driveR;
    this.front = front;
    this.back = back;

    this.navx = navx;

    setPIDSourceType(PIDSourceType.kDisplacement);
    pitchPID = new PIDController(climbP, climbI, climbD, this, this);
    pitchPID.setInputRange(-10.0, 10.0);
    pitchPID.setOutputRange(-1, 1);
    // pitchPID.setContinuous();
    pitchPID.setAbsoluteTolerance(0.5);
    disablePID();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
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


@Override
public void setPIDSourceType(PIDSourceType pidSource) {
  pidSourceType = pidSource;
}

@Override
public PIDSourceType getPIDSourceType() {
  return pidSourceType;
}

@Override
public double pidGet() {
  // if (pidSourceType == PIDSourceType.kRate)
  //     return navx.getRate();
  //   else
      return this.getPitch();
}

@Override
public void pidWrite(double output) {
  fClimb.set(normalizeOutput(-1.0 - output));
  bClimb.set(normalizeOutput(-1.0 + output));
}


  /**
   * Internal function to normalize a motor output
   * 
   * @param output the unnormalized output
   * @return the normalized output ranging from -1.0 to 1.0
   */

private static double normalizeOutput(double output) {
  if (output > 0.8)
    return 0.8;
  else if (output < -0.8)
    return -0.8;
  return output;
}

public void stop() {
  this.set(0, 0, 0);
  disablePID();
}

public void resetNavx() {
  navx.reset();
}

public void enablePID() {
    pitchPID.enable();
  }

  public void disablePID() {
    if (pitchPID.isEnabled()) {
      pitchPID.disable();
    }
  }

  public void setPID(double p, double i, double d) {
    this.pitchPID.setPID(p, i, d);
  }

  public boolean onTarget() {
    return pitchPID.onTarget();
  }

  private double normalizePitch(double pitch) {
    while (pitch >= 180) {
      pitch -= 360;
    }
    while (pitch <= -180) {
      pitch += 360;
    }
    return pitch;
  }

  public void setHeading(double angle, double direction) {
    this.pitchPID.setSetpoint(normalizePitch(angle));
    enablePID();
  }

  public double getPitch() {
    return normalizePitch(navx.getPitch());
  }

  public void getPIDController() {
    
  }

  public void setClimberPID(double p, double i, double d) {
    this.pClimber = p;
    this.iClimber = i;
    this.dClimber = d;
    this.getPIDController().setPID(p, i, d);
  }

  /**
   * Sets the P value for the internal PID controller for height.
   * 
   * @param p the proportional value
   */
  public void setClimberP(double p) {
    this.pClimber = p;
    setClimberPID(pClimber, iClimber, dClimber);
  }

  /**
   * Gets the proportional value for the internal PID controller for height.
   * 
   * @return the proportional value
   */
  public double getClimberP() {
    return pClimber;
  }

  /**
   * Sets the I value for the internal PID controller for height.
   * 
   * @param i the integral value
   */
  public void setClimberI(double i) {
    this.iClimber = i;
    setClimberPID(pClimber, iClimber, dClimber);
  }

  /**
   * Gets the integral value for the internal PID controller for height.
   * 
   * @return the integral value
   */
  public double getClimberI() {
    return iClimber;
  }

  /**
   * Sets the D value for the intenral PID controller for height.
   * 
   * @param d the derivative value
   */
  public void setClimberD(double d) {
    this.dClimber = d;
    setClimberPID(pClimber, iClimber, dClimber);
  }

  /**
   * Gets the derivative value for the internal PID controller for height.
   * 
   * @return the derivative value
   */
  public double getClimberD() {
    return dClimber;
  }
}

