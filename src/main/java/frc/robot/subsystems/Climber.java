/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDController;

import com.kauailabs.navx.frc.AHRS;

/**
 * Add your docs here.
 */
public class Climber extends SubsystemBase implements PIDSource, PIDOutput {
  public static final double HOLD = 0.21;

  private Spark fClimb, bClimb, driveR, driveL;

  private double pClimber, iClimber, dClimber;
  public static final double climbP = 0.2, climbI = 0.0, climbD = 0.0;
  private PIDController pitchPID;
  private PIDSourceType pidSourceType;
  private DigitalInput limit;

  private AHRS navx;

  public Climber(Spark fClimb, Spark bClimb, Spark driveL, Spark driveR, AHRS navx, DigitalInput limit) {
    this.fClimb = fClimb;
    this.bClimb = bClimb;
    this.driveL = driveL;
    this.driveR = driveR;
    this.limit = limit;

    this.navx = navx;

    setPIDSourceType(PIDSourceType.kDisplacement);
    pitchPID = new PIDController(climbP, climbI, climbD, this, this);
    pitchPID.setInputRange(-10.0, 10.0);
    pitchPID.setOutputRange(0, 1);
    pitchPID.setAbsoluteTolerance(0.5);
    disablePID();
  }

  @Override
  public void initDefaultCommand() {}

  public void setPinions(double front, double back) {
    if (getLS() && front > HOLD) {
      fClimb.set(0.21);
    } else {
      fClimb.set(-front);
    }

    bClimb.set(-back);
  }

  public void setDrive(double output) {
    driveL.set(output);
    driveR.set(-output);
  }

  public boolean getLS() {
    return limit.get();
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
    // return navx.getRate();
    // else
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
    if (output > 1)
      return 1;
    else if (output < -1)
      return -1;
    return output;
  }

  public void stop() {
    this.setPinions(0, 0);
    this.setDrive(0);
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
    this.pClimber = p;
    this.iClimber = i;
    this.dClimber = d;
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

  public void autoClimb() {
    this.pitchPID.setSetpoint(0); // Stay level
    enablePID();
  }

  public double getPitch() {
    return normalizePitch(navx.getPitch());
  }

  public void getPIDController() {

  }

  /**
   * Sets the P value for the internal PID controller for height.
   * 
   * @param p the proportional value
   */
  public void setClimberP(double p) {
    this.pClimber = p;
    setPID(pClimber, iClimber, dClimber);
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
    setPID(pClimber, iClimber, dClimber);
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
    setPID(pClimber, iClimber, dClimber);
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
