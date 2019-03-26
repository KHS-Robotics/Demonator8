package frc.robot.commands.tankdrive;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.TankDrive;
import frc.robot.vision.MoePiClient;
import frc.robot.vision.PixyCam;
import frc.robot.vision.MoePiClient.DeepSpaceVisionTarget;

public class VisionGetToTapeMoePi extends Command {
    public static final double kAngleOffset = 9.5;
    
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

            double offset;
            if(target.hasOnlyRight()) {
                offset = moepi.getAngle(kAngleOffset + 2.0) * 1.5; // TODO: verify if 2.0 is good enough
            } else {
                offset = moepi.getAngle(kAngleOffset) * 1.5;
            }

            drive.setHeading(drive.getHeading() + offset, -0.75);

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
        return (pixy.getLongestLine() != null && drive.getAverageUltrasonic() < 30) || this.isTimedOut();
    }
}
