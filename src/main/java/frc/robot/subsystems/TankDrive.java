/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.OI;
import frc.robot.commands.tankdrive.DriveWithJoystick;
import frc.robot.commands.tankdrive.DriveWithJoysticks;
import frc.robot.commands.tankdrive.DriveWithXbox;

public class TankDrive extends SubsystemBase implements PIDSource, PIDOutput {
  private double offset;
  private double P = 0.02, I = 0.0, D = 0.048;
  private double direction;

  private AHRS navx;
  private Encoder lEnc, rEnc;
  private PIDController yawPID;
  private PIDSourceType pidSourceType;

  private DoubleSolenoid Shifter;
  private WPI_VictorSPX FrontLeft, FrontRight, MiddleLeft, MiddleRight, RearLeft, RearRight;

  private static final Value HIGH_GEAR = Value.kForward, LOW_GEAR = Value.kReverse;
  private Value currentGear;

  private static final double driveLinDist = Math.PI * 6.0; //inches of circ of 6in wheel
  private static final int driveReducDen = 64; //64 spur 
  private static final int driveReducNum = 3 * 20; //3 times encoder gearing 20 tooth spur gearbox output
  private static final int driveCPR = 64; //counts per revolution of motor
  private static final double driveDistPP = (driveLinDist * driveReducNum) / (driveReducDen * driveCPR); //linear distance of the last stage arm per encoder pulse

  public TankDrive(WPI_VictorSPX fl, WPI_VictorSPX fr, WPI_VictorSPX ml, WPI_VictorSPX mr, WPI_VictorSPX rl, WPI_VictorSPX rr,
      DoubleSolenoid Shifter, AHRS navx, Encoder LeftDriveEnc, Encoder RightDriveEnc) {
    FrontLeft = fl;
    FrontRight = fr;
    MiddleLeft = ml;
    MiddleRight = mr;
    RearLeft = rl;
    RearRight = rr;
    lEnc = LeftDriveEnc;
    rEnc = RightDriveEnc;
    this.navx = navx;
    this.Shifter = Shifter;

    this.lEnc.setDistancePerPulse(driveDistPP);
    this.rEnc.setDistancePerPulse(driveDistPP);

    setPIDSourceType(PIDSourceType.kDisplacement);
    yawPID = new PIDController(P, I, D, this, this);
    yawPID.setInputRange(-180.0, 180.0);
    yawPID.setOutputRange(-1.0, 1.0);
    yawPID.setContinuous();
    yawPID.setAbsoluteTolerance(0.5);
    disablePID();
    shiftLow();
  }

  public void set(double left, double right) {
    left = (normalizeOutput(left));
    right = (normalizeOutput(right));

    FrontLeft.set(ControlMode.PercentOutput, left);
    FrontRight.set(ControlMode.PercentOutput, right);
    MiddleLeft.set(ControlMode.PercentOutput, left);
    MiddleRight.set(ControlMode.PercentOutput, right);
    RearLeft.set(ControlMode.PercentOutput, left);
    RearRight.set(ControlMode.PercentOutput, right);
  }

  /**
   * Internal function to normalize a motor output
   * 
   * @param output the unnormalized output
   * @return the normalized output ranging from -1.0 to 1.0
   */
  private static double normalizeOutput(double output) {
    if (output > 1)
      return 1.0;
    else if (output < -1)
      return -1.0;
    return output;
  }

  public void stop() {
    this.set(0, 0);
    disablePID();
  }

  public void setDirection(double direction) {
    this.direction = direction;
  }

  public void resetNavx() {
    navx.reset();
  }

  public void shiftLow() {
    if (LOW_GEAR.equals(currentGear)) {
      return;
    }
    Shifter.set(LOW_GEAR);
    currentGear = LOW_GEAR;
  }

  public void shiftHigh() {
    if (HIGH_GEAR.equals(currentGear)) {
      return;
    }
    Shifter.set(HIGH_GEAR);
    currentGear = HIGH_GEAR;
  }

  public void shift() {
    if (LOW_GEAR.equals(currentGear)) {
      shiftHigh();
    } else {
      shiftLow();
    }
  }

  /**
   * Gets the right drive encoder's distance
   * 
   * @return the right drive encoder's distance
   */
  public double getRightDistance() {
    return rEnc.getDistance();
  }

  /**
   * Gets the left drive encoder's distance
   * 
   * @return the left drive encoder's distance
   */
  public double getLeftDistance() {
    return lEnc.getDistance();
  }

  /**
   * Resets the encoders, should only be used when testing
   */
  public void resetEncoders() {
    lEnc.reset();
    rEnc.reset();
  }

  /**
   * Calculates the remaining distance the robot needs to drive before reaching
   * the desired distance
   * 
   * @param distance     the desired distance (in inches)
   * @param initialLeft  the initial left encoder distance (in inches, basically a
   *                     snapshot of {@link #getLeftDistance()})
   * @param initialRight the initial right encoder distance (in inches, basically
   *                     a snapshot of {@link #getRightDistance()})
   * @return the remaining distance the robot needs to drive
   */
  public double remainingDistance(double distance, double initialLeft, double initialRight) {
    final double CURRENT_RIGHT_VAL = Math.abs(getRightDistance());
    final double CURRENT_LEFT_VAL = Math.abs(getLeftDistance());
    final double DELTA_RIGHT = Math.abs(CURRENT_RIGHT_VAL - initialRight);
    final double DELTA_LEFT = Math.abs(CURRENT_LEFT_VAL - initialLeft);

    final double AVERAGE = (DELTA_RIGHT + DELTA_LEFT) / 2;

    final double REMAINING = distance - AVERAGE;

    return REMAINING;
  }

  public void enablePID() {
    yawPID.enable();
  }

  public void disablePID() {
    if (yawPID.isEnabled()) {
      yawPID.disable();
      direction = 0;
    }
  }

  public void setPID(double p, double i, double d) {
    this.yawPID.setPID(p, i, d);
  }

  public boolean onTarget() {
    return yawPID.onTarget();
  }

  private double normalizeYaw(double yaw) {
    while (yaw >= 180) {
      yaw -= 360;
    }
    while (yaw <= 180) {
      yaw += 360;
    }
    return yaw;
  }

  public void setHeading(double angle, double direction) {
    this.setDirection(direction);
    this.yawPID.setSetpoint(normalizeYaw(angle));
    enablePID();
  }

  public double getHeading() {
    return normalizeYaw(navx.getYaw());
  }

  public void setOffset(double angle) {
    this.offset = angle;
  }

  @Override
  public void pidWrite(double output) {
    double left = direction + output;
    double right = direction - output;

    this.set(left, right);
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

    if (pidSourceType == PIDSourceType.kRate)
      return navx.getRate();
    else
      return this.getHeading();
  }

  @Override
  public void initDefaultCommand() {
    OI oi = OI.getInstance();
    this.setDefaultCommand(new DriveWithJoystick(oi.drive, oi.leftJoystick));
  }
}
