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
import frc.robot.OI;
import frc.robot.commands.elevator.ElevateWithJoystick;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

public class Elevator extends PIDSubsystem {
  private double pElevator, iElevator, dElevator, pArm = 0.1, iArm = 0.01, dArm = 0.6, iZone = 5;
  private final double armTolerance = 1.0;
  private boolean shouldReset;
  private CANPIDController armPID;
  
  private static final Value CLOSE = Value.kReverse, OPEN = Value.kForward;
  private Value current;

  private Spark accL, accR;
  private WPI_VictorSPX elevator1, elevator2;
  private CANSparkMax arm;
  private CANEncoder armEncoder;
  private DigitalInput ls;
  private Encoder encoder;
  private DoubleSolenoid solenoid;

  private static final double eleLinDist = 18.0 * 0.25 * 2; //inches of .25in chain per output sprocket rev, doubled on the last stage
  private static final int eleReducNum = 4; //22 tooth sprocket gearbox output
  private static final int eleReducDen = 13; //64 planetary & 18 tooth sprocket
  private static final int eleCPR = 12; //counts per revolution of motor
  private static final double eleDistPP = (eleLinDist * eleReducNum) / (eleReducDen * eleCPR); //linear distance of the last stage arm per encoder pulse

  private static final int armReducNum = 18; //18 tooth sprocket gearbox output
  private static final int armReducDen = 7 * 5 * 32; //7, 5 planetary & 32 tooth sprocket
  private static final double armDegPR = (360.0 * armReducNum) / (armReducDen); //Arm deg per Neo rev

  public Elevator(WPI_VictorSPX elevator1, WPI_VictorSPX elevator2, Spark accL, Spark accR, CANSparkMax arm, DigitalInput ls, Encoder encoder,
      DoubleSolenoid solenoid) {
    super(0, 0, 0);
    this.elevator1 = elevator1;
    this.elevator2 = elevator2;
    this.accL = accL;
    this.accR = accR;
    this.arm = arm;
    this.ls = ls;
    this.encoder = encoder;
    this.solenoid = solenoid;

    this.arm.setIdleMode(IdleMode.kBrake);

    this.encoder.setDistancePerPulse(eleDistPP);

    this.armPID = new CANPIDController(arm);
    this.armPID.setOutputRange(-0.67, 0.67);
    setArmPID(pArm, iArm, dArm, iZone);

    this.armEncoder = this.arm.getEncoder();
    this.armEncoder.setPositionConversionFactor(armDegPR);
  }

  public boolean armOnTarget(double target) {
    return Math.abs(target - getArmRotation()) < armTolerance;
  }

  public double getArmRotation() {
    return armEncoder.getPosition() ;
  }

  public void setIntake(double l, double r) {
    accL.set(l);
    accR.set(-r);
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

  public double getElevatorHeight() {
    return returnPIDInput();
  }

  @Override
  protected void usePIDOutput(double output) {
    set(output);
  }

  public void stop() {
    disable();
    set(0);
    setIntake(0, 0);
    arm.stopMotor();
  }

  public void setElevatorPID(double p, double i, double d) {
    this.pElevator = p;
    this.iElevator = i;
    this.dElevator = d;
    this.getPIDController().setPID(p, i, d);
  }

  /**
   * Sets the P value for the internal PID controller for height.
   * 
   * @param p the proportional value
   */
  public void setElevatorP(double p) {
    this.pElevator = p;
    setElevatorPID(pElevator, iElevator, dElevator);
  }

  /**
   * Gets the proportional value for the internal PID controller for height.
   * 
   * @return the proportional value
   */
  public double getElevatorP() {
    return pElevator;
  }

  /**
   * Sets the I value for the internal PID controller for height.
   * 
   * @param i the integral value
   */
  public void setElevatorI(double i) {
    this.iElevator = i;
    setElevatorPID(pElevator, iElevator, dElevator);
  }

  /**
   * Gets the integral value for the internal PID controller for height.
   * 
   * @return the integral value
   */
  public double getElevatorI() {
    return iElevator;
  }

  /**
   * Sets the D value for the intenral PID controller for height.
   * 
   * @param d the derivative value
   */
  public void setElevatorD(double d) {
    this.dElevator = d;
    setElevatorPID(pElevator, iElevator, dElevator);
  }

  /**
   * Gets the derivative value for the internal PID controller for height.
   * 
   * @return the derivative value
   */
  public double getElevatorD() {
    return dElevator;
  }

  public void setArmPID(double p, double i, double d, double iZone) {
    this.pArm = p;
    this.iArm = i;
    this.dArm = d;
    this.iZone = iZone;
    armPID.setP(p);
    armPID.setI(i);
    armPID.setD(d);
    armPID.setIZone(iZone);
  }

  /**
   * Sets the P value for the internal PID controller for height.
   * 
   * @param p the proportional value
   */
  public void setArmP(double p) {
    this.pArm = p;
    setArmPID(pArm, iArm, dArm, iZone);
  }

  /**
   * Gets the proportional value for the internal PID controller for height.
   * 
   * @return the proportional value
   */
  public double getArmP() {
    return pArm;
  }

  /**
   * Sets the I value for the internal PID controller for height.
   * 
   * @param i the integral value
   */
  public void setArmI(double i) {
    this.iArm = i;
    setArmPID(pArm, iArm, dArm, iZone);
  }

  /**
   * Gets the integral value for the internal PID controller for height.
   * 
   * @return the integral value
   */
  public double getArmI() {
    return iArm;
  }

  /**
   * Sets the D value for the intenral PID controller for height.
   * 
   * @param d the derivative value
   */
  public void setArmD(double d) {
    this.dArm = d;
    setArmPID(pArm, iArm, dArm, iZone);
  }

  /**
   * Gets the derivative value for the internal PID controller for height.
   * 
   * @return the derivative value
   */
  public double getArmD() {
    return dArm;
  }
  

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ElevateWithJoystick(OI.getInstance().elevator, OI.getInstance().switchBox));
  }

  public void set(double output) {
    // if (!override && getLS() && output < 0)
      // output = 0;

    elevator1.set(-output);
    elevator2.set(output);
  }

  public void setArm(double output) {
    arm.set(output);
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

  public boolean getLS() {
    return ls.get();
  }

  public void setArmRotation(double angle) {
    armPID.setReference(angle, ControlType.kPosition);
  }
}
