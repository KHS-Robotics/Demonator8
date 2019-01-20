/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.TankDrive;

public class OI {

  public Joystick leftJoystick;
  public Joystick rightJoystick;

  private AHRS navx;
  private Encoder LeftDriveEnc, RightDriveEnc;

  private VictorSPX FrontLeft, FrontRight, MiddleLeft, MiddleRight, RearLeft, RearRight;
  private Spark intake;
  private DoubleSolenoid Shifter;

  public TankDrive drive;
  public CargoIntake cargoIntake;

  private OI() {

    leftJoystick = new Joystick(RobotMap.LEFT_DRIVE_STICK_PORT);
    rightJoystick = new Joystick(RobotMap.RIGHT_DRIVE_STICK_PORT);

    initCargoIntake();
    initDrive();
  }

  private static OI instance;

  public static OI getInstance() {
    if (instance == null) {
      instance = new OI();
    }
    return instance;
  }

  private void initDrive() {
    navx = new AHRS(RobotMap.NAVX_PORT, RobotMap.NAVX_UPDATE_RATE_HZ);

    LeftDriveEnc = new Encoder(RobotMap.LEFT_DRIVE_ENC_A, RobotMap.LEFT_DRIVE_ENC_B);
    RightDriveEnc = new Encoder(RobotMap.RIGHT_DRIVE_ENC_A, RobotMap.RIGHT_DRIVE_ENC_B);

    Shifter = new DoubleSolenoid(RobotMap.SHIFT_FORWARD_CHANNEL, RobotMap.SHIFT_REVERSE_CHANNEL);
    FrontLeft = new VictorSPX(RobotMap.FRONT_LEFT);
    FrontRight = new VictorSPX(RobotMap.FRONT_RIGHT);
    MiddleLeft = new VictorSPX(RobotMap.MIDDLE_LEFT);
    MiddleRight = new VictorSPX(RobotMap.MIDDLE_RIGHT);
    RearLeft = new VictorSPX(RobotMap.REAR_LEFT);
    RearRight = new VictorSPX(RobotMap.REAR_RIGHT);

    drive = new TankDrive(FrontLeft, FrontRight, MiddleLeft, MiddleRight, RearLeft, RearRight, Shifter, navx, LeftDriveEnc, RightDriveEnc);
  }

  private void initCargoIntake() {
    intake = new Spark(RobotMap.INTAKE);
    cargoIntake = new CargoIntake(intake);
  }
}
