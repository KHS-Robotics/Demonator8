/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Class to keep track of the button maps for teleop
 */
public class ButtonMap 
{
	/**
	 * Xbox Controller to control the Swerve Drive
	 */
	public static class DriveController
	{
		public static final int GO_STRAIGHT = XboxButton.kBumperRight.value;
		public static final int GO_TO_ZERO = XboxButton.kY.value;
		public static final int GO_TO_LEFT = XboxButton.kX.value;
		public static final int GO_TO_RIGHT = XboxButton.kB.value;
		public static final int GO_TO_180 = XboxButton.kA.value;
	}
	
	/**
	 * Switch Box to control the Accumualtor. Note that
	 * switches 3, 7, 8, 9, and 12 are reversed. Meaning,
	 * they read true when flipped down and false when flipped up.
	 */
	public static class SwitchBox
	{
        public static final int INTAKE_FORWARD = 6;
        public static final int INTAKE_REVERSE = 5;
        public static final int TOGGLE_ARMS = 12;
        public static final int ROTATE_ARM_FRONT = 0;
        public static final int ROTATE_ARM_BACK = 0;
		public static final int RELEASE = 0;
		public static final int RESET = 0;
		public static final int ELEVATE_CARGO_HIGH = 0;
		public static final int ELEVATE_CARGO_MEDIUM = 0;
		public static final int ELEVATE_CARGO_LOW = 0;
        public static final int ELEVATE_HATCH_HIGH = 0;
        public static final int ELEVATE_HATCH_MEDIUM = 0;
        public static final int ELEVATE_HATCH_LOW = 0;
		public static final int ELEVATOR_OVERIDE = 11;
		public static final int TUNE_PID = 0;
		public static final int F_CLIMBER_RAISE = 8;
		public static final int F_CLIMBER_LOWER = 7;
		public static final int B_CLIMBER_RAISE = 10;
		public static final int B_CLIMBER_LOWER = 9;
	}

	/**
	 * Represents a digital button on an XboxController
	 */
	 enum XboxButton {
		kBumperLeft(5),
		kBumperRight(6),
		kStickLeft(9),
		kStickRight(10),
		kA(1),
		kB(2),
		kX(3),
		kY(4),
		kBack(7),
		kStart(8);

		private int value;

		XboxButton(int value) {
			this.value = value;
		}
	}
}