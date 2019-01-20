/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.OI;
import frc.robot.commands.DriveWithJoysticks;

public class TankDrive extends SubsystemBase {
  private DoubleSolenoid Shifter;
  private VictorSPX FrontLeft, FrontRight, MiddleLeft, MiddleRight, RearLeft, RearRight;
  private AHRS navx;
  private static final Value HIGH_GEAR = Value.kForward, LOW_GEAR = Value.kReverse;
  private Value currentGear;

  public TankDrive(VictorSPX fl, VictorSPX fr, VictorSPX ml, VictorSPX mr, VictorSPX rl, VictorSPX rr, AHRS navx,
      DoubleSolenoid Shifter) {
    FrontLeft = fl;
    FrontRight = fr;
    MiddleLeft = ml;
    MiddleRight = mr;
    RearLeft = rl;
    RearRight = rr;
    this.navx = navx;
    this.Shifter = Shifter;
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
  }

  public void resetNavx() {
    navx.reset();
  }

  public double getHeading() {
    return normalizeYaw(navx.getYaw());
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

  @Override
  public void initDefaultCommand() {
    OI oi = OI.getInstance();
    this.setDefaultCommand(new DriveWithJoysticks(oi.drive, oi.leftJoystick, oi.rightJoystick));
  }
}
