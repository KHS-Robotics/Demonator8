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

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

public class Elevator extends PIDSubsystem {
  private boolean override, shouldReset;

  private CANPIDController armPID;
  private static final double kP = 0.00012, kI = 0.00000125, kD = 0.0004, kFF = 0.00004065;
  private static final double kMaxOutput = 1.0, kMinOutput = -1.0, kMaxVel = 4000, kMaxAcc = 4000, kAllowedArmErr = 3.0;
  
  private static final Value CLOSE = Value.kForward, OPEN = Value.kReverse;
  private Value current;

  private Spark accL, accR;
  private WPI_VictorSPX elevator1, elevator2;
  private CANSparkMax arm;
  private CANEncoder armEncoder;
  private DigitalInput ls;
  private Encoder encoder;
  private DoubleSolenoid solenoid;

  private static final double eleLinDist = 18.0 * 0.25 * 2; //inches of .25in chain per output sprocket rev, doubled on the last stage
  private static final int eleReducNum = 22 * 16; //22 tooth sprocket gearbox output, two motor gearbox 16 tooth sprocket
  private static final int eleReducDen = 20 * 18 * 52; //64 planetary & 18 tooth sprocket
  private static final int eleCPR = 12; //counts per revolution of motor
  private static final double eleDistPP = (eleLinDist * eleReducNum) / (eleReducDen * eleCPR); //linear distance of the last stage arm per encoder pulse

  private static final int armReducNum = 18; //18 tooth sprocket gearbox output
  private static final int armReducDen = 7 * 5 * 32; //7, 5 planetary & 32 tooth sprocket
  private static final double armDegPR = (360.0 * armReducNum) / (armReducDen); //Arm deg per Neo rev

  public Elevator(WPI_VictorSPX elevator1, WPI_VictorSPX elevator2, Spark accL, Spark accR, CANSparkMax arm, DigitalInput ls, Encoder encoder,
      DoubleSolenoid solenoid) {
    super(0.85, 0.003, 0.80);
    this.elevator1 = elevator1;
    this.elevator2 = elevator2;
    this.accL = accL;
    this.accR = accR;
    this.arm = arm;
    this.ls = ls;
    this.encoder = encoder;
    this.solenoid = solenoid;

    setNeutralMode(NeutralMode.Brake);
    this.arm.setIdleMode(IdleMode.kBrake);

    this.encoder.setDistancePerPulse(eleDistPP);

    armPID = arm.getPIDController();
    armPID.setOutputRange(kMinOutput, kMaxOutput);
    armPID.setSmartMotionMaxAccel(kMaxAcc, 0);
    armPID.setSmartMotionMaxVelocity(kMaxVel, 0);
    armPID.setSmartMotionAllowedClosedLoopError(kAllowedArmErr, 0);

    armPID.setP(kP);
    armPID.setI(kI);
    armPID.setD(kD);
    armPID.setFF(kFF);

    this.armEncoder = this.arm.getEncoder();
    this.armEncoder.setPositionConversionFactor(armDegPR);

    this.setAbsoluteTolerance(0.5);
    this.setOutputRange(-1, 1);
    this.setInputRange(0, 20);
  }

  public void setNeutralMode(NeutralMode mode) {
    elevator1.setNeutralMode(mode);
    elevator2.setNeutralMode(mode);
  }

  @Override
  public void setSetpoint(double setpoint) {
    super.setSetpoint(setpoint);
    this.enable();
  }

  public boolean armOnTarget(double target) {
    return Math.abs(target - getArmRotation()) < kAllowedArmErr;
  }

  public double getArmRotation() {
    return armEncoder.getPosition() ;
  }

  public void setIntake(double l, double r) {
    accL.set(l);
    accR.set(-r);
  }

  public void open() {
    // if(OPEN.equals(current))
    //   return;

      solenoid.set(OPEN);
      current = OPEN;
  }

  public void close() {
    // if(CLOSE.equals(current))
    //   return;

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
    setArm(0);
  }

  public void setElevatorPID(double p, double i, double d) {
    this.getPIDController().setPID(p, i, d);
  }

  /**
   * Sets the P value for the internal PID controller for height.
   * 
   * @param p the proportional value
   */
  public void setElevatorP(double p) {
    setElevatorPID(p, this.getPIDController().getI(), this.getPIDController().getD());
  }

  /**
   * Gets the proportional value for the internal PID controller for height.
   * 
   * @return the proportional value
   */
  public double getElevatorP() {
    return this.getPIDController().getP();
  }

  /**
   * Sets the I value for the internal PID controller for height.
   * 
   * @param i the integral value
   */
  public void setElevatorI(double i) {
    setElevatorPID(this.getPIDController().getP(), i, this.getPIDController().getD());
  }

  /**
   * Gets the integral value for the internal PID controller for height.
   * 
   * @return the integral value
   */
  public double getElevatorI() {
    return this.getPIDController().getI();
  }

  /**
   * Sets the D value for the intenral PID controller for height.
   * 
   * @param d the derivative value
   */
  public void setElevatorD(double d) {
    setElevatorPID(this.getPIDController().getP(), this.getPIDController().getI(), d);
  }

  /**
   * Gets the derivative value for the internal PID controller for height.
   * 
   * @return the derivative value
   */
  public double getElevatorD() {
    return this.getPIDController().getD();
  }

  public void setOverride(boolean set) {
    this.override = set;
  }
  
  /**
   * @return the override
   */
  public boolean isOverride() {
    return override;
  }


  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ElevateWithJoystick(OI.getInstance().elevator, OI.getInstance().switchBox));
  }

  public void set(double output) {
    if (!override && getLS() && output < 0)
      output = 0;

    elevator1.set(normalize(-output));
    elevator2.set(normalize(output));
  }

  public void setArm(double output) {
    armPID.setReference(normalize(output)*kMaxVel, ControlType.kVelocity);
  }

  public boolean armCollision(double output) {
    final double FREE_ROTATION_MIN = 7.79;

    final double FRONT_BODY_COLLISION = -300;
    final double BACK_INTAKE_COLLISION = -90;

    double currentHeight = getElevatorHeight();
    double armDegree = getArmRotation();

    boolean canMove = false;
    
    if(currentHeight >= FREE_ROTATION_MIN) {
      canMove = true;
    } else {

      if(output > 0 && armDegree > BACK_INTAKE_COLLISION) {
        canMove = false;
      }

      if(output < 0 && armDegree < FRONT_BODY_COLLISION) {
        canMove = false;
      }
    }

    return canMove;
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
    armPID.setReference(angle, ControlType.kSmartMotion);
  }

  private static double normalize(double value) {
    if(value > 1)
      return 1;
    
    if(value < -1)
      return -1;
    
    return value;
  }
}
