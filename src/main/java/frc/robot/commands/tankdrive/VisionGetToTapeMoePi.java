package frc.robot.commands.tankdrive;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.TankDrive;
import frc.robot.vision.MoePiClient;
import frc.robot.vision.PixyCam;

public class VisionGetToTapeMoePi extends Command {
    public static final double kAngleOffset = 5.0; // TODO: find a good angle offset empirically

    private boolean foundTarget;

    private TankDrive drive;
    private MoePiClient moepi;
    private PixyCam pixy;
    
    public VisionGetToTapeMoePi(TankDrive drive, MoePiClient moepi, PixyCam pixy) {
        super(3);

        this.drive = drive;
        this.moepi = moepi;
        this.pixy = pixy;

        this.requires(drive);
    }
    
    @Override
    protected void initialize() {
        foundTarget = false;
        drive.setLight(true);
    }

    @Override
    protected void execute() {
        if(!foundTarget && !moepi.getBoxes().isEmpty()) {
            drive.setHeading(drive.getHeading() + moepi.getAngle(kAngleOffset), -1);
            foundTarget = true;
        }
    }

    @Override
    protected void end() {
        drive.setLight(false);
        drive.stop();
    }

    @Override
    protected boolean isFinished() {
        return pixy.getLongestLine() != null || this.isTimedOut();
    }
}
