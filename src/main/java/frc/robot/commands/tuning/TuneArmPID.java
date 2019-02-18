/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.tuning;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator;

public class TuneArmPID extends Command {
    private Elevator elevator;

    public TuneArmPID(Elevator elevator) {
        this.requires(elevator);
        this.elevator = elevator;
    }

    @Override
    protected void initialize() {
        SmartDashboard.putNumber("Arm-P", 0);
        SmartDashboard.putNumber("Arm-I", 0);
        SmartDashboard.putNumber("Arm-D", 0);
        SmartDashboard.putNumber("Arm-IZone", 0);
        SmartDashboard.putNumber("Arm-Setpoint", -34);
    }

    @Override
    protected void execute() {
        double p = SmartDashboard.getNumber("Arm-P", 0.0);
        double i = SmartDashboard.getNumber("Arm-I", 0.0);
        double d = SmartDashboard.getNumber("Arm-D", 0.0);
        double iZone = SmartDashboard.getNumber("Arm-IZone", 0.0);
        elevator.setArmPID(p, i, d, iZone);

        //double setpoint = SmartDashboard.getNumber("Arm-Setpoint", elevator.getArmRotation());
        //elevator.setArmRotation(setpoint, 0);
    }

    @Override
    protected void end() {
        elevator.stop();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}