/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.logging.Logger;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Climber;

public class OI {

  public Joystick leftJoystick;
  public Joystick rightJoystick;

  private AHRS navx;
  private Encoder LeftDriveEnc, RightDriveEnc;

  private VictorSPX FrontLeft, FrontRight, MiddleLeft, MiddleRight, RearLeft, RearRight;
  private Spark intake;
  private DoubleSolenoid Shifter;

  private Spark fl, fr, ml, mr, rl, rr;
  private DigitalInput frontLS, backLS;

  private Spark elevatorMotor, arm, elevatorIntake;
  private DigitalInput elevatorLS;
  private Encoder elevatorEncoder;
  private DoubleSolenoid elevatorSolenoid;

  public TankDrive drive;
  public CargoIntake cargoIntake;
  public Elevator elevator;
  public Climber climber;

  private OI() {

    leftJoystick = new Joystick(RobotMap.LEFT_DRIVE_STICK_PORT);
    rightJoystick = new Joystick(RobotMap.RIGHT_DRIVE_STICK_PORT);

    initCargoIntake();
    initDrive();
    initClimber();
    initElevator();
  }

  private static OI instance;

  public static OI getInstance() {
    if (instance == null) {
      instance = new OI();
    }
    return instance;
  }

  private void initDrive() {

    try {

      Logger.info("Initializing Tank Drive...");

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

      drive = new TankDrive(FrontLeft, FrontRight, MiddleLeft, MiddleRight, RearLeft, RearRight, Shifter, navx,
        LeftDriveEnc, RightDriveEnc);
    } catch(Exception ex) {
        Logger.error("Failed to initialize Tank Drive!", ex);
    }
  }

  private void initCargoIntake() {

    try {

      Logger.info("Initializing Cargo Intake...");

      intake = new Spark(RobotMap.INTAKE);
      cargoIntake = new CargoIntake(intake);
    } catch(Exception ex) {
      Logger.error("Failed to initialize Cargo Intake!", ex);
    }
  }

  private void initClimber() {

    try {

      Logger.info("Initializing Climber...");

      fl = new Spark(RobotMap.FRONT_LEFT);
      fr = new Spark(RobotMap.FRONT_RIGHT);
      ml = new Spark(RobotMap.MIDDLE_LEFT);
      mr = new Spark(RobotMap.MIDDLE_RIGHT);
      rl = new Spark(RobotMap.REAR_LEFT);
      rr = new Spark(RobotMap.REAR_RIGHT);

      frontLS = new DigitalInput(RobotMap.FRONT_LIMIT_SWITCH);
      backLS = new DigitalInput(RobotMap.BACK_LIMIT_SWITCH);

      climber = new Climber(fl, fr, ml, mr, rl, rr, frontLS, backLS);
    } catch(Exception ex) {
        Logger.error("Failed to initialize Cimber!", ex);
    }
  }

  private void initElevator() {

    try {

      Logger.info("Initializing Elevator...");

      elevatorMotor = new Spark(RobotMap.ELEVATOR_MOTOR);
      arm = new Spark(RobotMap.ARM);
      elevatorIntake = new Spark(RobotMap.ELEVATOR_INTAKE);
      elevatorLS = new DigitalInput(RobotMap.ELEVATOR_LIMIT_SWITCH);
      elevatorEncoder = new Encoder(RobotMap.ELEVATOR_ENCODER_A, RobotMap.ELEVATOR_ENCODER_B);
      elevatorSolenoid = new DoubleSolenoid(RobotMap.ELEVATOR_SOLENOID_A, RobotMap.ELEVATOR_SOLENOID_B);

      elevator = new Elevator(elevatorMotor, arm, elevatorIntake, elevatorLS, elevatorEncoder, elevatorSolenoid);
    } catch(Exception ex) {
        Logger.error("Failed to initialize Elevator!", ex);
    }
  }

}
