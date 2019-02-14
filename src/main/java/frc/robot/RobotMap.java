/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SPI;

public class RobotMap {
  // Joysticks
  public static final int LEFT_DRIVE_STICK_PORT = 0;
  public static final int RIGHT_DRIVE_STICK_PORT = 1;
  public static final int SWITCH_BOX = 2;
  // public static final int XBOX_CONTROLLER = 0;

  // Tankdrive
  public static final int FRONT_LEFT = 13;
  public static final int MIDDLE_LEFT = 14;
  public static final int REAR_LEFT = 15;

  public static final int FRONT_RIGHT = 0;
  public static final int MIDDLE_RIGHT = 1;
  public static final int REAR_RIGHT = 2;

  public static final int ULTRASONIC_L = 0;
  public static final int ULTRASONIC_R = 1;

  public static final int RIGHT_DRIVE_ENC_A = 0;
  public static final int RIGHT_DRIVE_ENC_B = 1;
  public static final int LEFT_DRIVE_ENC_A = 2;
  public static final int LEFT_DRIVE_ENC_B = 3;

  public static final int SHIFT_FORWARD_CHANNEL = 0;
  public static final int SHIFT_REVERSE_CHANNEL = 1;

  // Intake
  public static final int INTAKE = 8;

  // Navx
  public static final SPI.Port NAVX_PORT = SPI.Port.kMXP;
  public static final byte NAVX_UPDATE_RATE_HZ = (byte) 50;

  // Climber
  public static final int FRONT_LIMIT_SWITCH = 8;
  public static final int BACK_LIMIT_SWITCH = 7;
  public static final int F_CLIMB = 0;
  public static final int B_CLIMB = 1;
  public static final int CLIMB_DRIVE_L = 2;
  public static final int CLIMB_DRIVE_R = 3;

  // Elevator
  public static final int ELEVATOR_MOTOR = 9;
  public static final int ARM = 9;
  public static final int ELEVATOR_ACC_L = 4;
  public static final int ELEVATOR_ACC_R = 5;

  public static final int ELEVATOR_ENCODER_A = 4;
  public static final int ELEVATOR_ENCODER_B = 5;

  public static final int ELEVATOR_SOLENOID_A = 2;
  public static final int ELEVATOR_SOLENOID_B = 3;

  public static final int ELEVATOR_LIMIT_SWITCH = 6;
}
