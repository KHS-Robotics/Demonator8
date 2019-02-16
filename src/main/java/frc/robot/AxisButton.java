/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

public class AxisButton extends Button {
    private double value, range;
    private Joystick stick;
    private int axis;

    public AxisButton(double value, double range, Joystick stick, int axis) {
        super();
        this.value = value;
        this.range = range;
        this.stick = stick;
        this.axis = axis;
    }

    @Override
    public boolean get() {
        double val = stick.getRawAxis(axis);
        return (value - range) < val && (value + range) > val;
    }
}
