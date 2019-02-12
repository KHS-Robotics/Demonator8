package frc.robot.commands.tankdrive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.TankDrive;

public class TuneDrivePID extends Command {
    private TankDrive drive;

    public TuneDrivePID(TankDrive drive) {
        this.requires(drive);
        this.drive = drive;
    }

    @Override
    protected void initialize() {
        SmartDashboard.putNumber("Drive-P", 0);
        SmartDashboard.putNumber("Drive-I", 0);
        SmartDashboard.putNumber("Drive-D", 0);
        SmartDashboard.putNumber("Drive-Setpoint", 0);
    }

    @Override
    protected void execute() {
        double p = SmartDashboard.getNumber("Drive-P", 0.0);
        double i = SmartDashboard.getNumber("Drive-I", 0.0);
        double d = SmartDashboard.getNumber("Drive-D", 0.0);
        drive.setPID(p, i, d);

        double setpoint = SmartDashboard.getNumber("Drive-Setpoint", drive.getHeading());
        drive.setHeading(setpoint, 0);
    }

    @Override
    protected void end() {
        drive.stop();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
