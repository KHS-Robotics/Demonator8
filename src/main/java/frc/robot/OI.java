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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.subsystems.Accumulator;
import frc.robot.subsystems.TankDrive;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public final AHRS navx;
  public final Joystick leftJoystick;
  public final Joystick rightJoystick;
  public final VictorSPX FrontLeft, FrontRight, MiddleLeft, MiddleRight, RearLeft, RearRight;
  public final Spark intake;
  public final DoubleSolenoid Shifter;

  public final TankDrive drive;
  public final Accumulator accumulator;


  private OI(){

   navx = new AHRS(RobotMap.NAVX_PORT, RobotMap.NAVX_UPDATE_RATE_HZ);
   leftJoystick = new Joystick(RobotMap.LEFT_DRIVE_STICK_PORT);
   rightJoystick = new Joystick(RobotMap.RIGHT_DRIVE_STICK_PORT);
   Shifter = new DoubleSolenoid(RobotMap.SHIFT_FORWARD_CHANNEL, RobotMap.SHIFT_REVERSE_CHANNEL);
   FrontLeft = new VictorSPX(RobotMap.FRONT_LEFT);
   FrontRight = new VictorSPX(RobotMap.FRONT_RIGHT);
   MiddleLeft = new VictorSPX(RobotMap.MIDDLE_LEFT);
   MiddleRight = new VictorSPX(RobotMap.MIDDLE_RIGHT);
   RearLeft = new VictorSPX(RobotMap.REAR_LEFT);
   RearRight = new VictorSPX(RobotMap.REAR_RIGHT);
   intake = new Spark(RobotMap.INTAKE);
   accumulator = new Accumulator(intake);
   drive = new TankDrive(FrontLeft, FrontRight, MiddleLeft, MiddleRight, RearLeft, RearRight, navx, Shifter);

  }
  private static OI instance;

  public static OI getInstance() 
  {
    if(instance == null)
    {
      instance = new OI();
    }
    return instance;
  }
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
