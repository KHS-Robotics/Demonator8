/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SubsystemBase;

public class StopSubsystem extends InstantCommand {
    SubsystemBase[] subsystem;
    Elevator elev;

  public StopSubsystem(SubsystemBase... subsystem) {
    super();
    this.subsystem = subsystem;
    for(SubsystemBase sub: subsystem) {
      requires(sub);
    }
  }

  public StopSubsystem(Elevator elev, SubsystemBase... subsystem) {
    super();
    this.subsystem = subsystem;
    this.elev = elev;
    for(SubsystemBase sub: subsystem) {
      requires(sub);
    }
    requires(elev);
  }

  @Override
  protected void initialize() {
    for(SubsystemBase sub : subsystem) {
      sub.stop();
    }

    if(elev != null) {
      elev.stop();
    }
  }

}
