package frc.robot.commands.tankdrive;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.TankDrive;

public class TimedBrake extends Command {
    private TankDrive drive;

    public TimedBrake(TankDrive drive, double time) {
        super(time);
    }

    @Override
    protected void initialize() {
        drive.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    protected void end() {
        drive.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    protected boolean isFinished() {
        return this.isTimedOut();
    }
}
