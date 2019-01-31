/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;

public class CargoIntake extends SubsystemBase {
	private boolean enabled;

	private Spark motor;

	public CargoIntake(Spark motor) {
		this.motor = motor;
	}

	@Override
	public void stop() {
		motor.set(0);
	}
	
	public void intake() {
		motor.set(1);
	}

	public void reverse() {
		motor.set(-1);
	}
}