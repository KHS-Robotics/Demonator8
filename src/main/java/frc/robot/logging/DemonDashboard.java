/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.logging;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.commands.tankdrive.VisionGetToTapeMoePi;

/**
 * Class to put data to the SmartDashboard on a separate thread
 */
public class DemonDashboard {
    private DemonDashboard() {
    }

    private static boolean running;

    /**
     * Starts the <code>DemonDashboard</code> on a new thread.
     */
    public static void start() {
        if (running)
            return;

        Logger.info("Starting DemonDashboard...");
        new DemonDashboardThread().start();

        running = true;
    }

    /**
     * Stops the <code>DemonDashboard</code>
     */
    public static void stop() {
        running = false;
    }

    /**
     * The magic behind this class...
     */
    private static class DemonDashboardThread extends Thread implements Runnable {
        private static final OI oi = OI.getInstance();

        /**
         * Puts data to the SmartDashboard every 50ms. The data is retrieved from OI
         * 
         * @see org.usfirst.frc.team4342.robot.OI
         */
        @Override
        public void run() {
            SmartDashboard.putBoolean("DemonDashboard", true);
            
            while (running) {
                try {
                    if (oi.drive != null) {
                        SmartDashboard.putNumber("Drive-Heading", oi.drive.getHeading());
                        SmartDashboard.putNumber("Roll", oi.drive.getRoll());
                        SmartDashboard.putNumber("Pitch", oi.drive.getPitch());

                        SmartDashboard.putNumber("Left Ultrasonic", oi.drive.getLeftUltrasonic());
                        SmartDashboard.putNumber("Right Ultrasonic", oi.drive.getRightUltrasonic());

                        SmartDashboard.putNumber("Left Distance", oi.drive.getLeftDistance());
                        SmartDashboard.putNumber("Right Distance", oi.drive.getRightDistance());

                        if(oi.udp != null) {
                            SmartDashboard.putNumber("MoePi NumBoxes", oi.udp.getBoxes().size());
                            SmartDashboard.putNumber("MoePi NumDeepSpaceTargets", oi.udp.getTargets().size());
                            SmartDashboard.putBoolean("MoePi Sees DeepSpaceTarget", oi.udp.hasTarget());

                            double angle = oi.udp.getAngle();
                            SmartDashboard.putNumber("MoePi CamAngle", angle);
                            SmartDashboard.putNumber("MoePi BotAngle", angle + VisionGetToTapeMoePi.kAngleOffset);
                        }

                        if(oi.pixy != null) {
                            SmartDashboard.putBoolean("Pixy Tape", oi.pixy.getLongestLine() != null);
                            SmartDashboard.putNumber("Pixy NumLines", oi.pixy.getNumLines());
                        }

                        if(oi.udp != null && oi.pixy != null) {
                            SmartDashboard.putBoolean("VISION", oi.pixy.getLongestLine() != null || oi.udp.hasTarget());
                        }
                    }

                    if (oi.elevator != null) {
                        SmartDashboard.putNumber("Elev-Height", oi.elevator.getElevatorHeight());
                        SmartDashboard.putBoolean("Elev-LS", oi.elevator.getLS());

                        SmartDashboard.putNumber("Arm Rotation", oi.elevator.getArmRotation());
                    }

                    if(oi.climber != null) {
                        SmartDashboard.putBoolean("Climber-LS", oi.climber.getLS());
                    }

                    // SmartDashboard.putNumber("Analog Button: ",
                    // OI.getInstance().switchBox.getRawAxis(3));

                    Thread.sleep(50);
                } catch (Exception ex) {
                    Logger.error("DemonDashboard crashed!", ex);
                    DemonDashboard.stop();
                }
            }

            SmartDashboard.putBoolean("DemonDashboard", false);
        }
    }
}