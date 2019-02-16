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

public class TuneElevatorPID extends Command {
  private Elevator elevator;

  public TuneElevatorPID(Elevator elevator) {
      this.elevator = elevator;
      
      this.requires(elevator);
  }
  
  @Override
  protected void initialize() {
    SmartDashboard.putNumber("Elevator-P", SmartDashboard.getNumber("Elevator-P", 0.0));
      SmartDashboard.putNumber("Elevator-I", SmartDashboard.getNumber("Elevator-I", 0.0));
      SmartDashboard.putNumber("Elevator-D", SmartDashboard.getNumber("Elevator-D", 0.0));
      SmartDashboard.putNumber("Elevator-Setpoint", SmartDashboard.getNumber("Elevator-Setpoint", 0.0));
  }

  @Override
    protected void execute() {
        double p = SmartDashboard.getNumber("Elevator-P", elevator.getElevatorP());
        double i = SmartDashboard.getNumber("Elevator-I", elevator.getElevatorI());
        double d = SmartDashboard.getNumber("Elevator-D", elevator.getElevatorD());
        elevator.setElevatorPID(p, i, d);

        double setpoint = SmartDashboard.getNumber("Elevator-Setpoint", elevator.getElevatorHeight());
        elevator.setSetpoint(setpoint);
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