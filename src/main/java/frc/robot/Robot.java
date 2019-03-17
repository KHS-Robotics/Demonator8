/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auton.CrossHabLine;
import frc.robot.auton.ScoreHatchFrontCargoShip;
import frc.robot.logging.DemonDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static boolean isEnabled;
  public static OI m_oi;

  CommandGroup m_autonomousCommand;
  // SendableChooser<CommandGroup> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = OI.getInstance();
  
    DemonDashboard.start();

    // m_chooser.setDefaultOption("Nothing", null);
    // m_chooser.addOption("Forward Cross Hab Line (intake side)", new CrossHabLine(m_oi.drive, 135, 0, 1));
    // m_chooser.addOption("Backward Cross Hab Line (vision side)", new CrossHabLine(m_oi.drive, 135, 0, -1));
    // m_chooser.addOption("Score Hatch Cargo Ship Front", new ScoreHatchFrontCargoShip(m_oi.drive, m_oi.udp, m_oi.pixy, m_oi.elevator));
    // SmartDashboard.putData(m_chooser);

    SmartDashboard.putNumber("Autonomous", 0);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    isEnabled = false;
    m_oi.drive.setNeutralMode(NeutralMode.Brake);
    Scheduler.getInstance().run();
    Scheduler.getInstance().removeAll();
    m_oi.drive.setLight(false);
  }

  @Override
  public void disabledPeriodic() {
    // if(OI.getInstance().leftJoystick.getRawButton(2)) {
    //   OI.getInstance().drive.resetNavx();
    // } 
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    isEnabled = true;
    m_oi.drive.setNeutralMode(NeutralMode.Coast);
    //m_autonomousCommand = m_chooser.getSelected();
    OI.getInstance().drive.resetEncoders();
    OI.getInstance().drive.resetNavx();
    OI.getInstance().elevator.reset();

    OI.getInstance().elevator.open();

    final int autonNumber = (int) SmartDashboard.getNumber("Autonomous", 0);
    if(autonNumber == 1) {
      m_autonomousCommand = new CrossHabLine(m_oi.drive, 135, 0, 1);
    }
    else if(autonNumber == 2) {
      m_autonomousCommand = new CrossHabLine(m_oi.drive, 135, 0, -1);
    }
    else if(autonNumber == 3) {
      m_autonomousCommand =  new ScoreHatchFrontCargoShip(m_oi.drive, m_oi.udp, m_oi.pixy, m_oi.elevator);
    }
    else {
      m_autonomousCommand = null;
    }

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // check for auton cancel
    if(OI.getInstance().rightJoystick.getRawButton(ButtonMap.RightJoystick.CANCEL_AUTO) && m_autonomousCommand != null && m_autonomousCommand.isRunning()) {
      m_autonomousCommand.cancel();
    }

    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    isEnabled = true;
    m_oi.drive.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // check for auton cancel
    if(OI.getInstance().rightJoystick.getRawButton(ButtonMap.RightJoystick.CANCEL_AUTO) && m_autonomousCommand != null && m_autonomousCommand.isRunning()) {
      m_autonomousCommand.cancel();
    }

    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public static boolean enabled() {
    return isEnabled;
  }
}
