/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.BothIntake;
import frc.robot.commands.SetDefaultCommand;
import frc.robot.commands.StopSubsystem;
import frc.robot.commands.climber.ClimbWithThrottle;
import frc.robot.commands.climber.HoldFrontClimb;
import frc.robot.commands.climber.LowerAll;
import frc.robot.commands.climber.LowerBack;
import frc.robot.commands.climber.LowerFront;
import frc.robot.commands.climber.RaiseAll;
import frc.robot.commands.climber.ClimbPID;
import frc.robot.commands.climber.RaiseBack;
import frc.robot.commands.climber.RaiseFront;
import frc.robot.commands.climber.StartClimbDrive;
import frc.robot.commands.elevator.ElevateCargoToHigh;
import frc.robot.commands.elevator.ElevateCargoToLow;
import frc.robot.commands.elevator.ElevateCargoToMedium;
import frc.robot.commands.elevator.ElevateToHatchHigh;
import frc.robot.commands.elevator.ElevateToHatchLow;
import frc.robot.commands.elevator.ElevateToHatchMiddle;
import frc.robot.commands.elevator.ElevateWithJoystick;
import frc.robot.commands.elevator.GrabBall;
import frc.robot.commands.elevator.OverrideElevator;
import frc.robot.commands.elevator.RotateArm;
import frc.robot.commands.elevator.StartGrab;
import frc.robot.commands.elevator.StopElevator;
import frc.robot.commands.elevator.StopRelease;
import frc.robot.commands.elevator.ToggleArm;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.commands.intake.StartIntake;
import frc.robot.commands.tankdrive.DriveStraightAtTargetJoystick;
import frc.robot.commands.tankdrive.DriveStraightJoystick;
import frc.robot.commands.tankdrive.Shift;
import frc.robot.commands.tankdrive.ShiftHigh;
import frc.robot.commands.tankdrive.ShiftHighDriveStraight;
import frc.robot.commands.tankdrive.ShiftLow;
import frc.robot.commands.tankdrive.ToggleLight;
import frc.robot.commands.tuning.TuneArmPID;
import frc.robot.commands.tuning.TuneClimberPID;
import frc.robot.commands.tuning.TuneDrivePID;
import frc.robot.commands.tuning.TuneElevatorPID;
import frc.robot.logging.Logger;
import frc.robot.commands.tankdrive.XOffsetVision;

import java.net.SocketException;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.TankDrive;
import frc.robot.vision.MoePiClient;
import frc.robot.vision.PixyCam;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Climber;

public class OI {

  public Joystick leftJoystick;
  public Joystick rightJoystick;
  public XboxController DriveController;
  public Joystick switchBox;

  private AHRS navx;
  private Encoder LeftDriveEnc, RightDriveEnc;
  private AnalogInput rUltra, lUltra;

  private WPI_VictorSPX Left1, Left2, Right1, Right2, elevator1, elevator2;
  private Spark intake;
  private DoubleSolenoid Shifter;

  private Spark fClimb, bClimb, climbDriveL, climbDriveR;
  private DigitalInput climberLS;

  private Spark elevatorAccL, elevatorAccR;
  private DigitalInput elevatorLS;
  private Encoder elevatorEncoder;
  private DoubleSolenoid elevatorSolenoid;
  private CANSparkMax arm;

  private DigitalOutput VisionLight;

  public TankDrive drive;
  public CargoIntake cargoIntake;
  public Elevator elevator;
  public Climber climber;
  public MoePiClient udp;
  public PixyCam pixy;

  private OI() {

    leftJoystick = new Joystick(RobotMap.LEFT_DRIVE_STICK_PORT);
    rightJoystick = new Joystick(RobotMap.RIGHT_DRIVE_STICK_PORT);
    switchBox = new Joystick(RobotMap.SWITCH_BOX);

    initElevator();
    initCargoIntake();
    initDrive();
    initClimber();
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

      lUltra = new AnalogInput(RobotMap.ULTRASONIC_L);
      rUltra = new AnalogInput(RobotMap.ULTRASONIC_R);

      Shifter = new DoubleSolenoid(RobotMap.SHIFT_FORWARD_CHANNEL, RobotMap.SHIFT_REVERSE_CHANNEL);
      Left1 = new WPI_VictorSPX(RobotMap.DRIVE_LEFT1);
      Left2 = new WPI_VictorSPX(RobotMap.DRIVE_LEFT2);
      Right1 = new WPI_VictorSPX(RobotMap.DRIVE_RIGHT1);
      Right2 = new WPI_VictorSPX(RobotMap.DRIVE_RIGHT2);

      VisionLight = new DigitalOutput(RobotMap.VISION_LIGHT);
      VisionLight.set(false);

      try {
        pixy = new PixyCam();
      } catch(Exception ex) {
        ex.printStackTrace();
      }

      drive = new TankDrive(Left1, Left2, Right1, Right2, Shifter, navx, LeftDriveEnc, RightDriveEnc, lUltra, rUltra, VisionLight);

      JoystickButton shift = new JoystickButton(rightJoystick, ButtonMap.RightJoystick.TOGGLE_GEAR);
      shift.whenPressed(new ShiftHigh(drive));
      shift.whenReleased(new ShiftLow(drive));

      JoystickButton goStraight = new JoystickButton(rightJoystick, ButtonMap.RightJoystick.GO_STRAIGHT);
      goStraight.whenPressed(new DriveStraightJoystick(drive, rightJoystick));
      goStraight.whenReleased(new StopSubsystem(drive));

      JoystickButton highGearGoStraightButton = new JoystickButton(rightJoystick, ButtonMap.RightJoystick.HIGHGEAR_GO_STRAIGHT);
      highGearGoStraightButton.whenPressed(new ShiftHighDriveStraight(rightJoystick, drive));
      highGearGoStraightButton.whenReleased(new ShiftLow(drive));

      JoystickButton visionAlign = new JoystickButton(rightJoystick, ButtonMap.RightJoystick.VISION_ALIGN);
      visionAlign.whenPressed(new XOffsetVision(drive, udp, pixy));
      visionAlign.whenReleased(new StopSubsystem(drive));

      // JoystickButton driveStraightTarget = new JoystickButton(leftJoystick, ButtonMap.LeftJoystick.DRIVE_AT_TARGET);
      // driveStraightTarget.whenPressed(new DriveStraightAtTargetJoystick(drive, udp, leftJoystick));
      // driveStraightTarget.whenReleased(new StopSubsystem(drive));

      // JoystickButton tunedrivePIDButton = new JoystickButton(switchBox, 2);
      // tunedrivePIDButton.whenPressed(new TuneDrivePID(drive));
      // tunedrivePIDButton.whenReleased(new StopSubsystem(drive));
      
      try {
        udp = new MoePiClient(drive);
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
      intakeForward.whenPressed(new BothIntake(elevator, cargoIntake));
      intakeForward.whenReleased(new StopSubsystem(elevator, cargoIntake));
      
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

      fClimb = new Spark(RobotMap.F_CLIMB);
      bClimb = new Spark(RobotMap.B_CLIMB);
      climbDriveL = new Spark(RobotMap.CLIMB_DRIVE_L);
      climbDriveR = new Spark(RobotMap.CLIMB_DRIVE_R);
      climberLS = new DigitalInput(RobotMap.CLIMB_LS);

      climber = new Climber(fClimb, bClimb, climbDriveL, climbDriveR, navx, climberLS);

      JoystickButton raiseFrontClimb = new JoystickButton(leftJoystick, ButtonMap.LeftJoystick.F_CLIMBER_RAISE);
      raiseFrontClimb.whenPressed(new RaiseFront(climber));
      raiseFrontClimb.whenReleased(new StopSubsystem(climber));

      JoystickButton lowerFrontClimb = new JoystickButton(leftJoystick, ButtonMap.LeftJoystick.F_CLIMBER_LOWER);
      lowerFrontClimb.whenPressed(new LowerFront(climber));
      lowerFrontClimb.whenReleased(new HoldFrontClimb(climber));

      JoystickButton raiseBackClimb = new JoystickButton(leftJoystick, ButtonMap.LeftJoystick.B_CLIMBER_RAISE);
      raiseBackClimb.whenPressed(new RaiseBack(climber));
      raiseBackClimb.whenReleased(new StopSubsystem(climber));

      JoystickButton lowerBackClimb = new JoystickButton(leftJoystick, ButtonMap.LeftJoystick.B_CLIMBER_LOWER);
      lowerBackClimb.whenPressed(new LowerBack(climber));
      lowerBackClimb.whenReleased(new StopSubsystem(climber));

      JoystickButton climb = new JoystickButton(leftJoystick, ButtonMap.LeftJoystick.LOWER_ALL);
      climb.whenPressed(new ClimbPID(climber));
      climb.whenReleased(new HoldFrontClimb(climber));

      JoystickButton climbDrive = new JoystickButton(rightJoystick, ButtonMap.RightJoystick.CLIMB_DRIVE);
      climbDrive.whenPressed(new StartClimbDrive(climber, 1.0, 0.21));
      climbDrive.whenReleased(new HoldFrontClimb(climber));

      // JoystickButton tuneClimbPID = new JoystickButton(switchBox, 8);
      // tuneClimbPID.whenPressed(new TuneClimberPID(climber, switchBox));
      // tuneClimbPID.whenReleased(new StopSubsystem(climber));

      // JoystickButton climbThrottle = new JoystickButton(switchBox, 2);
      // climbThrottle.whenPressed(new ClimbWithThrottle(leftJoystick, rightJoystick, climber));
      // climbThrottle.whenReleased(new StopSubsystem(climber));

    } catch(Exception ex) {
        Logger.error("Failed to initialize Cimber!", ex);
    }
  }

  private void initElevator() {

    try {

      Logger.info("Initializing Elevator...");

      elevator1 = new WPI_VictorSPX(RobotMap.ELEVATOR_MOTOR1);
      elevator2 = new WPI_VictorSPX(RobotMap.ELEVATOR_MOTOR2);

      arm = new CANSparkMax(RobotMap.ARM, MotorType.kBrushless);
      elevatorAccL = new Spark(RobotMap.ELEVATOR_ACC_L);
      elevatorAccR = new Spark(RobotMap.ELEVATOR_ACC_R);
      elevatorLS = new DigitalInput(RobotMap.ELEVATOR_LIMIT_SWITCH);
      elevatorEncoder = new Encoder(RobotMap.ELEVATOR_ENCODER_A, RobotMap.ELEVATOR_ENCODER_B);
      elevatorSolenoid = new DoubleSolenoid(RobotMap.ELEVATOR_SOLENOID_A, RobotMap.ELEVATOR_SOLENOID_B);

      elevator = new Elevator(elevator1, elevator2, elevatorAccL, elevatorAccR, arm, elevatorLS, elevatorEncoder, elevatorSolenoid);
      
      // Button to set the elevator to the high cargo port
			AxisButton elevateCargoHigh = new AxisButton(switchBox, ButtonMap.SwitchBox.ELEVATE_CARGO_HIGH, ButtonMap.SwitchBox.BUTTON_AXIS);
			elevateCargoHigh.whenPressed(new ElevateCargoToHigh(elevator));
			elevateCargoHigh.whenReleased(new StopElevator(elevator));

			// Button to set the elevator to the medium cargo port
			AxisButton elevateCargoMedium = new AxisButton(switchBox, ButtonMap.SwitchBox.ELEVATE_CARGO_MEDIUM, ButtonMap.SwitchBox.BUTTON_AXIS);
			elevateCargoMedium.whenPressed(new ElevateCargoToMedium(elevator));
			elevateCargoMedium.whenReleased(new StopElevator(elevator));
			
			// Button to set the elevator to the low cargo part
			AxisButton elevateCargoLow = new AxisButton(switchBox, ButtonMap.SwitchBox.ELEVATE_CARGO_LOW, ButtonMap.SwitchBox.BUTTON_AXIS);
			elevateCargoLow.whenPressed(new ElevateCargoToLow(elevator));
			elevateCargoLow.whenReleased(new StopElevator(elevator));
      
      // Button to set the elevator to the high hatch port
			AxisButton elevateHatchHigh = new AxisButton(switchBox, ButtonMap.SwitchBox.ELEVATE_HATCH_HIGH, ButtonMap.SwitchBox.BUTTON_AXIS);
			elevateHatchHigh.whenPressed(new ElevateToHatchHigh(elevator));
			elevateHatchHigh.whenReleased(new StopElevator(elevator));

			// Button to set the elevator to the medium hatch port
			AxisButton elevateHatchMedium = new AxisButton(switchBox, ButtonMap.SwitchBox.ELEVATE_HATCH_MEDIUM, ButtonMap.SwitchBox.BUTTON_AXIS);
			elevateHatchMedium.whenPressed(new ElevateToHatchMiddle(elevator));
			elevateHatchMedium.whenReleased(new StopElevator(elevator));
			
			// Button to set the elevator to the low hatch part
			AxisButton elevateHatchLow = new AxisButton(switchBox, ButtonMap.SwitchBox.ELEVATE_HATCH_LOW, ButtonMap.SwitchBox.BUTTON_AXIS);
			elevateHatchLow.whenPressed(new ElevateToHatchLow(elevator));
      elevateHatchLow.whenReleased(new StopElevator(elevator));
      
      // Button to toggle arms
			JoystickButton toggleArms = new JoystickButton(switchBox, ButtonMap.SwitchBox.TOGGLE_ARMS);
			toggleArms.whenPressed(new ToggleArm(elevator));

      // Button to use overrided elevator controls
      JoystickButton overrideButton = new JoystickButton(switchBox, ButtonMap.SwitchBox.ELEVATOR_OVERIDE);
      overrideButton.whenPressed(new SetDefaultCommand(elevator, new ElevateWithJoystick(elevator, switchBox))); // Button is inverted
      overrideButton.whenReleased(new SetDefaultCommand(elevator, new OverrideElevator(switchBox, elevator)));

      JoystickButton pointForward = new JoystickButton(switchBox, ButtonMap.SwitchBox.ROTATE_ARM_FRONT);
      pointForward.whenPressed(new RotateArm(elevator, -90));
      pointForward.whenReleased(new StopElevator(elevator));

      JoystickButton pointBackward = new JoystickButton(switchBox, ButtonMap.SwitchBox.ROTATE_ARM_BACK);
      pointBackward.whenPressed(new RotateArm(elevator, -315));
      pointBackward.whenReleased(new StopElevator(elevator));

      AxisButton grabAngle = new AxisButton(switchBox, ButtonMap.SwitchBox.GRAB, ButtonMap.SwitchBox.BUTTON_AXIS);
      grabAngle.whenPressed(new GrabBall(elevator));
      grabAngle.whenReleased(new StopElevator(elevator));

      AxisButton positiveNinety = new AxisButton(switchBox, ButtonMap.SwitchBox.POSITIVE_NINETY, ButtonMap.SwitchBox.BUTTON_AXIS);
      positiveNinety.whenPressed(new RotateArm(elevator, 90));
      positiveNinety.whenReleased(new StopElevator(elevator));


      // JoystickButton tuneArm = new JoystickButton(switchBox, 2);
      // tuneArm.whenPressed(new TuneArmPID(elevator));
      // tuneArm.whenReleased(new StopElevator(elevator));

      // JoystickButton tuneElev = new JoystickButton(switchBox, 5);
      // tuneElev.whenPressed(new TuneElevatorPID(elevator));
      // tuneElev.whenPressed(new StopElevator(elevator));

    } catch(Exception ex) {
        Logger.error("Failed to initialize Elevator!", ex);
    }
  }

}
