/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import frc.robot.subsystems.Elevator;

/**
 * Add your docs here.
 */
public class ElevateToHatchLow extends Elevate {
    private static final double HATCH_LOW_HEIGHT = 0;

    public ElevateToHatchLow(Elevator elevator) {
        super(elevator, HATCH_LOW_HEIGHT);
    }
}
