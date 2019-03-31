package frc.robot.commands.tankdrive;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.logging.Logger;
import frc.robot.subsystems.TankDrive;
import frc.robot.vision.MoePiClient;
import frc.robot.vision.PixyCam;

public class VisionGetToTapeMoePi extends Command {
    public static final double kAngleOffset = 9.5;

    public static final double PIXY_X_OFFSET = 11.5;
    public static final double PIXY_Y_OFFSET = 14.0;
    public static final double TAPE_Y_OFFSET = 18.0;
    public static final double TOTAL_Y_OFFSET = PIXY_Y_OFFSET + TAPE_Y_OFFSET;

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
        //drive.setLight(true);
    }

    @Override
    protected void execute() {
        //getToEdgeofTape();
        dumbMethodOfApproach();
    }

    protected void getToEdgeofTape() {
        if(!foundTarget && moepi.getBoxes().size() >= 2) {
            double moepiAngle = moepi.getAngle();
            double moepiDistance = moepi.getDistance();

            Logger.debug("moepiAngle = " + moepiAngle + ", moepiDistance = " + moepiDistance);
            
            double x = moepiDistance*Math.sin(Math.toRadians(moepiAngle));
            double y = moepiDistance*Math.cos(Math.toRadians(moepiAngle));

            double targetX = x + PIXY_X_OFFSET;
            double targetY = y - TOTAL_Y_OFFSET;

            double angle = Math.toDegrees(Math.atan2(targetX, targetY));
            drive.setHeading(drive.getHeading() + angle, 0.4);

            Logger.debug("X = " + x + ", Y = " + y);
            Logger.debug("targetX = " + targetX + ", targetY = " + targetY);
            Logger.debug("angle = " + angle);

            foundTarget = true;
            //drive.setLight(false);
        }
    }
    
    protected void dumbMethodOfApproach() {
        if(!foundTarget && moepi.getBoxes().size() >= 2) {
            double angle = moepi.getAngle(kAngleOffset) * 1.5;

            drive.setHeading(drive.getHeading() + angle, 0.4);
            foundTarget = true;
            //drive.setLight(false);
        }
        else if(!foundTarget && moepi.getBoxes().size() == 1 && MoePiClient.Box.TargetType.RIGHT.value == moepi.getBoxes().get(0).type) {
            double angle = moepi.getAngle(kAngleOffset + 2) * 1.5;
            
            drive.setHeading(drive.getHeading() + angle, 0.4);
            foundTarget = true;
            //drive.setLight(false);
        }
    }

    @Override
    protected void end() {
        //drive.setLight(false);
        drive.stop();
    }

    @Override
    protected boolean isFinished() {
        return (pixy.getLongestLine() != null && drive.getAverageUltrasonic() < 30) || this.isTimedOut();
    }
}
