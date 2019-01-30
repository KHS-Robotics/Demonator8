/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class Elevator extends PIDSubsystem {
  private double p, i, d;
  private boolean override, shouldReset;

  private static final Value CLOSE = Value.kReverse, OPEN = Value.kForward;
  private Value current;

  private Spark elevatormotor, intake, arm;
  private DigitalInput ls;
  private Encoder encoder;
  private DoubleSolenoid solenoid;

  public Elevator(Spark elevatormotor, Spark intake, Spark arm, DigitalInput ls, Encoder encoder,
      DoubleSolenoid solenoid) {
    super(0, 0, 0);
    this.elevatormotor = elevatormotor;
    this.intake = intake;
    this.arm = arm;
    this.ls = ls;
    this.encoder = encoder;
    this.solenoid = solenoid;
  }

  public void setIntake(double output) {
    intake.set(output);
  }

  public void open() {
    if(OPEN.equals(current))
      return;

      solenoid.set(OPEN);
      current = OPEN;
  }

  public void close() {
    if(CLOSE.equals(current))
      return;

      solenoid.set(CLOSE);
      current = CLOSE;
  }

  public void toggle(){
    if (CLOSE.equals(current)) {
      open();
    } else {
      close();
    }
  }

  @Override
  public void disable() {
    if (this.getPIDController().isEnabled())
      super.disable();
  }

  @Override
  protected double returnPIDInput() {
    return encoder.getDistance();
  }

  @Override
  protected void usePIDOutput(double output) {
    set(output);
  }

  public void stop() {
    disable();
    set(0);
  }

  public void setPID(double p, double i, double d) {
    this.p = p;
    this.i = i;
    this.d = d;
    this.getPIDController().setPID(p, i, d);
  }

  /**
   * Sets the P value for the internal PID controller for height.
   * 
   * @param p the proportional value
   */
  public void setP(double p) {
    this.p = p;
    setPID(p, i, d);
  }

  /**
   * Gets the proportional value for the internal PID controller for height.
   * 
   * @return the proportional value
   */
  public double getP() {
    return p;
  }

  /**
   * Sets the I value for the internal PID controller for height.
   * 
   * @param i the integral value
   */
  public void setI(double i) {
    this.i = i;
    setPID(p, i, d);
  }

  /**
   * Gets the integral value for the internal PID controller for height.
   * 
   * @return the integral value
   */
  public double getI() {
    return i;
  }

  /**
   * Sets the D value for the intenral PID controller for height.
   * 
   * @param d the derivative value
   */
  public void setD(double d) {
    this.d = d;
    setPID(p, i, d);
  }

  /**
   * Gets the derivative value for the internal PID controller for height.
   * 
   * @return the derivative value
   */
  public double getD() {
    return d;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void set(double output) {
    if (!override && getLS() && output < 0)
      elevatormotor.set(0);

    elevatormotor.set(output);
  }

  public void setOverride(boolean flag) {
    override = flag;
  }

  /**
   * Resets the elevator's encoder
   */
  public void reset() {
    encoder.reset();
  }

  public void setResetFlag(boolean flag) {
    shouldReset = flag;
  }

  /**
   * Gets if the elevator is at the bottom
   * 
   * @return true if the elevator is at the bottom, false otherwise
   */
  public boolean isAtBottom() {
    return getLS() && shouldReset;
  }

  public boolean getLS() {
    return ls.get();
  }

}
