/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.StopSubsystem;
import frc.robot.commands.elevator.ElevateCargoToHigh;
import frc.robot.commands.elevator.ElevateCargoToLow;
import frc.robot.commands.elevator.ElevateCargoToMedium;
import frc.robot.commands.elevator.ElevateToHatchHigh;
import frc.robot.commands.elevator.ElevateToHatchLow;
import frc.robot.commands.elevator.ElevateToHatchMiddle;
import frc.robot.commands.elevator.StopElevator;
import frc.robot.commands.elevator.ToggleArm;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.commands.intake.StartIntake;
import frc.robot.logging.Logger;

import java.net.SocketException;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.TankDrive;
import frc.robot.vision.UDPTracker;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Climber;

public class OI {

  public Joystick leftJoystick;
  public Joystick rightJoystick;
  public XboxController DriveController;
  public Joystick switchBox;

  private AHRS navx;
  private Encoder LeftDriveEnc, RightDriveEnc;

  private WPI_VictorSPX FrontLeft, FrontRight, MiddleLeft, MiddleRight, RearLeft, RearRight;
  private Spark intake;
  private DoubleSolenoid Shifter;

  private Spark fl, fr, ml, mr, rl, rr;
  private DigitalInput frontLS, backLS;

  private Spark elevatorMotor, elevatorIntake;
  private DigitalInput elevatorLS;
  private Encoder elevatorEncoder;
  private DoubleSolenoid elevatorSolenoid;
  private CANSparkMax arm;

  public TankDrive drive;
  public CargoIntake cargoIntake;
  public Elevator elevator;
  public Climber climber;
  public UDPTracker udp;

  private OI() {

    leftJoystick = new Joystick(RobotMap.LEFT_DRIVE_STICK_PORT);
    rightJoystick = new Joystick(RobotMap.RIGHT_DRIVE_STICK_PORT);
    DriveController = new XboxController(RobotMap.XBOX_CONTROLLER);
    switchBox = new Joystick(RobotMap.SWITCH_BOX);

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
      FrontLeft = new WPI_VictorSPX(RobotMap.FRONT_LEFT);
      FrontRight = new WPI_VictorSPX(RobotMap.FRONT_RIGHT);
      MiddleLeft = new WPI_VictorSPX(RobotMap.MIDDLE_LEFT);
      MiddleRight = new WPI_VictorSPX(RobotMap.MIDDLE_RIGHT);
      RearLeft = new WPI_VictorSPX(RobotMap.REAR_LEFT);
      RearRight = new WPI_VictorSPX(RobotMap.REAR_RIGHT);

      drive = new TankDrive(FrontLeft, FrontRight, MiddleLeft, MiddleRight, RearLeft, RearRight, Shifter, navx,
        LeftDriveEnc, RightDriveEnc);

      
      try {
        udp = new UDPTracker(drive, "MoePi", 5810);
      } catch (SocketException ex) {
        ex.printStackTrace();
      }

    } catch(Exception ex) {
      Logger.error("Failed to initialize Tank Drive!", ex);
    }
  }

  private void initCargoIntake() {

    try {

      Logger.info("Initializing Cargo Intake...");

      intake = new Spark(RobotMap.INTAKE);
      cargoIntake = new CargoIntake(intake);

      // Button to start intake
			JoystickButton intakeForward = new JoystickButton(switchBox, ButtonMap.SwitchBox.INTAKE_FORWARD);
			intakeForward.whenPressed(new StartIntake(cargoIntake));
      intakeForward.whenReleased(new StopSubsystem(cargoIntake));
      
      // Button to reverse intake
			JoystickButton intakeReverse = new JoystickButton(switchBox, ButtonMap.SwitchBox.INTAKE_REVERSE);
			intakeReverse.whenPressed(new ReverseIntake(cargoIntake));
			intakeReverse.whenReleased(new StopSubsystem(cargoIntake));

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
      arm = new CANSparkMax(RobotMap.ARM, MotorType.kBrushless);
      elevatorIntake = new Spark(RobotMap.ELEVATOR_INTAKE);
      elevatorLS = new DigitalInput(RobotMap.ELEVATOR_LIMIT_SWITCH);
      elevatorEncoder = new Encoder(RobotMap.ELEVATOR_ENCODER_A, RobotMap.ELEVATOR_ENCODER_B);
      elevatorSolenoid = new DoubleSolenoid(RobotMap.ELEVATOR_SOLENOID_A, RobotMap.ELEVATOR_SOLENOID_B);

      elevator = new Elevator(elevatorMotor, elevatorIntake, arm, elevatorLS, elevatorEncoder, elevatorSolenoid);
    
      // Button to set the elevator to the high cargo port
			JoystickButton elevateCargoHigh = new JoystickButton(switchBox, ButtonMap.SwitchBox.ELEVATE_CARGO_HIGH);
			elevateCargoHigh.whenPressed(new ElevateCargoToHigh(elevator));
			elevateCargoHigh.whenReleased(new StopElevator(elevator));

			// Button to set the elevator to the medium cargo port
			JoystickButton elevateCargoMedium = new JoystickButton(switchBox, ButtonMap.SwitchBox.ELEVATE_CARGO_MEDIUM);
			elevateCargoMedium.whenPressed(new ElevateCargoToMedium(elevator));
			elevateCargoMedium.whenReleased(new StopElevator(elevator));
			
			// Button to set the elevator to the low cargo part
			JoystickButton elevateCargoLow = new JoystickButton(switchBox, ButtonMap.SwitchBox.ELEVATE_CARGO_LOW);
			elevateCargoLow.whenPressed(new ElevateCargoToLow(elevator));
			elevateCargoLow.whenReleased(new StopElevator(elevator));
      
      // Button to set the elevator to the high hatch port
			JoystickButton elevateHatchHigh = new JoystickButton(switchBox, ButtonMap.SwitchBox.ELEVATE_HATCH_HIGH);
			elevateHatchHigh.whenPressed(new ElevateToHatchHigh(elevator));
			elevateHatchHigh.whenReleased(new StopElevator(elevator));

			// Button to set the elevator to the medium hatch port
			JoystickButton elevateHatchMedium = new JoystickButton(switchBox, ButtonMap.SwitchBox.ELEVATE_HATCH_MEDIUM);
			elevateHatchMedium.whenPressed(new ElevateToHatchMiddle(elevator));
			elevateHatchMedium.whenReleased(new StopElevator(elevator));
			
			// Button to set the elevator to the low hatch part
			JoystickButton elevateHatchLow = new JoystickButton(switchBox, ButtonMap.SwitchBox.ELEVATE_HATCH_LOW);
			elevateHatchLow.whenPressed(new ElevateToHatchLow(elevator));
      elevateHatchLow.whenReleased(new StopElevator(elevator));
      
      // Button to toggle arms
			JoystickButton toggleArms = new JoystickButton(switchBox, ButtonMap.SwitchBox.TOGGLE_ARMS);
			toggleArms.whenPressed(new ToggleArm(elevator));
      toggleArms.whenReleased(new StopElevator(elevator));

    } catch(Exception ex) {
        Logger.error("Failed to initialize Elevator!", ex);
    }
  }

}
