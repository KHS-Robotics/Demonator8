package frc.robot.commands.tankdrive;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.TankDrive;
import frc.robot.vision.MoePiClient;
import frc.robot.vision.PixyCam;
import frc.robot.vision.MoePiClient.DeepSpaceVisionTarget;

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
        drive.stop();
        drive.setLight(true);
        
        foundTarget = false;
    }

    @Override
    protected void execute() {
        if(!foundTarget && moepi.hasTarget()) {
            DeepSpaceVisionTarget target = moepi.getCenterTarget();

            if(target.hasOnlyRight()) {
                drive.setHeading(drive.getHeading() + moepi.getAngle(kAngleOffset - 2.0), -0.75);
            } else {
                drive.setHeading(drive.getHeading() + moepi.getAngle(kAngleOffset), -0.75);
            }

            drive.setLight(false);
            foundTarget = true;
        }
    }

    @Override
    protected void end() {
        drive.stop();
        drive.setLight(false);
    }

    @Override
    protected boolean isFinished() {
        return pixy.getLongestLine() != null || this.isTimedOut();
    }
}
