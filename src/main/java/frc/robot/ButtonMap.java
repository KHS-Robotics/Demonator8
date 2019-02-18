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
	 * Switch Box to control the Accumualtor. Note that
	 * switch 11 is reversed. Meaning,
	 * it reads true when flipped down and false when flipped up.
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
		public static final double ELEVATE_CARGO_HIGH = -0.65;
		public static final double ELEVATE_CARGO_MEDIUM = 0;
		public static final double ELEVATE_CARGO_LOW = 0.35;
        public static final double ELEVATE_HATCH_HIGH = -0.21;
        public static final double ELEVATE_HATCH_MEDIUM = -0.375;
        public static final double ELEVATE_HATCH_LOW = 0.67;
		public static final int ELEVATOR_OVERIDE = 11;
		public static final int TUNE_PID = 0;
		public static final int F_CLIMBER_RAISE = 8;
		public static final int F_CLIMBER_LOWER = 7;
		public static final int B_CLIMBER_RAISE = 10;
		public static final int B_CLIMBER_LOWER = 9;

		public static final int BUTTON_AXIS = 3;
	}

	public static class RightJoystick
	{
		public static final int HIGHGEAR_GO_STRAIGHT = 1;
		public static final int TOGGLE_GEAR = 5;
		public static final int GO_STRAIGHT = 4;
	}

	public static class LeftJoystick
	{
		public static final int LIFT_ALL = 1;
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