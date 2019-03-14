package frc.robot.commands.tankdrive;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.TankDrive;
import frc.robot.vision.MoePiClient;
import frc.robot.vision.PixyCam;

public class VisionAlignTarget extends CommandGroup {
    public VisionAlignTarget(TankDrive drive, MoePiClient moepi, PixyCam pixy) {
        this.addSequential(new VisionGetToTapeMoePi(drive, moepi, pixy));
        this.addSequential(new VisionAlignTargetPixy(drive, pixy));
    }
}
