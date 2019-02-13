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
    private double uLim;
    private double lLim;
    private Joystick stick;
    private int axis;

    public AxisButton(double uLim, double lLim, Joystick stick, int axis) {
        super();
        this.uLim = uLim;
        this.lLim = lLim;
        this.stick = stick;
        this.axis = axis;
    }

    @Override
    public boolean get() {
        double val = stick.getRawAxis(axis);
        return val >= lLim && val < uLim;
    }
}
