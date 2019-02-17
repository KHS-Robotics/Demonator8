/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.tankdrive;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.TankDrive;

public class GoToAngle extends InstantCommand {
    private static final int MAX_ON_TARGET_COUNT = 10;
    private int onTargetCounter;
    protected TankDrive drive;
    protected double angle;

  public GoToAngle(TankDrive drive, double angle) {
    super();
    this.drive = drive;
    this.angle = angle;
    this.requires(drive);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    drive.setHeading(angle, 0);
    onTargetCounter = 0;
  }
  
  @Override
  protected boolean isFinished(){
    if(drive.onTarget()) {
      onTargetCounter++;
    }
    return onTargetCounter > MAX_ON_TARGET_COUNT || this.isTimedOut();
  }
  @Override
  protected void end() {
    drive.stop();
  }

}
