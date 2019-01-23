/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;

public class RobotMap {
  // Joysticks
  public static final int LEFT_DRIVE_STICK_PORT = 0;
  public static final int RIGHT_DRIVE_STICK_PORT = 1;

  // Pneumatics
  public static final int SHIFT_FORWARD_CHANNEL = 0;
  public static final int SHIFT_REVERSE_CHANNEL = 0;

  // Tankdrive motors
  public static final int FRONT_LEFT = 0;
  public static final int FRONT_RIGHT = 0;
  public static final int MIDDLE_LEFT = 0;
  public static final int MIDDLE_RIGHT = 0;
  public static final int REAR_LEFT = 0;
  public static final int REAR_RIGHT = 0;

  public static final int INTAKE = 0;

  // Navx
  public static final SPI.Port NAVX_PORT = SPI.Port.kMXP;
  public static final byte NAVX_UPDATE_RATE_HZ = (byte) 50;

  // Encoders
  public static final int RIGHT_DRIVE_ENC_A = 0;
  public static final int RIGHT_DRIVE_ENC_B = 0;
  public static final int LEFT_DRIVE_ENC_A = 0;
  public static final int LEFT_DRIVE_ENC_B = 0;

  // Climber
  public static final int FRONT_LIMIT_SWITCH = 0;
  public static final int BACK_LIMIT_SWITCH = 0;

  // Elevator
  public static final int ELEVATOR_MOTOR = 0;
  public static final int ARM = 0;
  public static final int ELEVATOR_INTAKE = 0;
  public static final int ELEVATOR_ENCODER_A = 0;
  public static final int ELEVATOR_ENCODER_B = 0;
  public static final int ELEVATOR_SOLENOID_A = 0;
  public static final int ELEVATOR_SOLENOID_B = 0;
  public static final int ELEVATOR_LIMIT_SWITCH = 0;
}
