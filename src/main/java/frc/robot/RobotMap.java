/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalSource;
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
  public static final DigitalSource RIGHT_DRIVE_ENC_A = null;
  public static final DigitalSource RIGHT_DRIVE_ENC_B = null;
  public static final DigitalSource LEFT_DRIVE_ENC_A = null;
  public static final DigitalSource LEFT_DRIVE_ENC_B = null;
}
