package frc.robot.commands.tankdrive;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.TankDrive;
import frc.robot.vision.MoePiClient;
import frc.robot.vision.PixyCam;

public class VisionGetToTapeMoePi extends Command {
    public static final double kAngleOffset = 9.5; // TODO: find a good angle offset empirically

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
        drive.stop();
        drive.setLight(true);
    }

    @Override
    protected void execute() {
        if(!foundTarget && moepi.getBoxes().size() >= 2) {
            drive.setHeading(drive.getHeading() + moepi.getAngle(kAngleOffset), -0.75);
            foundTarget = true;
            drive.setLight(false);
        }
        else if(!foundTarget && moepi.getBoxes().size() == 1 && MoePiClient.Box.TargetType.RIGHT.value == moepi.getBoxes().get(0).type) {
            drive.setHeading(drive.getHeading() + moepi.getAngle(kAngleOffset-2), -0.75);
            foundTarget = true;
            drive.setLight(false);
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
