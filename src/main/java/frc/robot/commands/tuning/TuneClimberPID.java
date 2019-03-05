/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.tuning;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Climber;

public class TuneClimberPID extends Command {
  private Climber climber;

  public TuneClimberPID(Climber climber) {
      this.climber = climber;
      
      this.requires(climber);
  }
  
  @Override
  protected void initialize() {
    SmartDashboard.putNumber("Climber-P", SmartDashboard.getNumber("Climber-P", 0.0));
      SmartDashboard.putNumber("Climber-I", SmartDashboard.getNumber("Climber-I", 0.0));
      SmartDashboard.putNumber("Climber-D", SmartDashboard.getNumber("Climber-D", 0.0));
      SmartDashboard.putNumber("Climber-Setpoint", SmartDashboard.getNumber("Climber-Setpoint", 0.0));
  }

  @Override
    protected void execute() {
        double p = SmartDashboard.getNumber("Climber-P", climber.getClimberP());
        double i = SmartDashboard.getNumber("Climber-I", climber.getClimberI());
        double d = SmartDashboard.getNumber("Climber-D", climber.getClimberD());
        climber.setPID(p, i, d);

        double setpoint = SmartDashboard.getNumber("Climber-Setpoint", climber.getPitch());
        climber.setPitch(setpoint);
    }

    @Override
    protected void end() {
        climber.stop();
    }

  @Override
  protected boolean isFinished() {
    return false;
  }
}