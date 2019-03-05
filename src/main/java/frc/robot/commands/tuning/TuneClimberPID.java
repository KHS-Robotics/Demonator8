/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.tuning;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.climber.HoldFrontClimb;
import frc.robot.subsystems.Climber;

public class TuneClimberPID extends Command {
  private Climber climber;
  private boolean go = false, end = false;
  private Joystick stick;

  public TuneClimberPID(Climber climber, Joystick stick) {
      this.climber = climber;
      this.stick = stick;
      
      this.requires(climber);
  }
  
  @Override
  protected void initialize() {
    SmartDashboard.putNumber("Climber-P", SmartDashboard.getNumber("Climber-P", 0.0));
      SmartDashboard.putNumber("Climber-I", SmartDashboard.getNumber("Climber-I", 0.0));
      SmartDashboard.putNumber("Climber-D", SmartDashboard.getNumber("Climber-D", 0.0));
      // SmartDashboard.putNumber("Climber-Setpoint", SmartDashboard.getNumber("Climber-Setpoint", 0.0));
      SmartDashboard.putBoolean("GO", false);
      SmartDashboard.putBoolean("END", false);
  }

  @Override
    protected void execute() {
        double p = SmartDashboard.getNumber("Climber-P", climber.getClimberP());
        double i = SmartDashboard.getNumber("Climber-I", climber.getClimberI());
        double d = SmartDashboard.getNumber("Climber-D", climber.getClimberD());
        climber.setPID(p, i, d);

        if(go) {
          climber.autoClimb();
        }

        go = stick.getRawButton(10);
        end = stick.getRawButton(9);
        // double setpoint = SmartDashboard.getNumber("Climber-Setpoint", climber.getPitch());
    }

    @Override
    protected void end() {
      climber.stop();
      //Scheduler.getInstance().add(new HoldFrontClimb(climber));
    }

  @Override
  protected boolean isFinished() {
    return end;
  }
}