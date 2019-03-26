/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import java.util.Arrays;

import edu.wpi.first.wpilibj.SPI;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2Line;
import io.github.pseudoresonance.pixy2api.Pixy2Line.Vector;
import io.github.pseudoresonance.pixy2api.links.Link;
import io.github.pseudoresonance.pixy2api.links.SPILink;

import frc.robot.logging.Logger;

/**
 * <p>Class for the <a href="https://pixycam.com/pixy2/">Pixy2</a> to get gaffer 
 * tape (line) data for Destination: Deep Space.</p>
 * 
 * @see io.github.pseudoresonance.pixy2api.Pixy2
 * @see io.github.pseudoresonance.pixy2api.Pixy2Line
 */
public class PixyCam {
    /** "Right-most" x-value for Pixy's Line Tracking. */
    public static final int CAMERA_MAX_X = 78;
    /** "Lower-most" y-value for Pixy's Line Tracking. */
    public static final int CAMERA_MAX_Y = 51;

    public static final double CAMERA_HEIGHT = 4.5;
    public static final double PIXY_LENS = 63;
    public static final double VERT_FOV = 40;
    public static final double HORIZ_FOV = 60;
    public static final double VERT_D_PER_P = VERT_FOV / CAMERA_MAX_Y;
    public static final double HORIZ_D_PER_P = HORIZ_FOV / CAMERA_MAX_X;

    private int currentNumLines;
    private double lineAngle;
    private double lineLength;

    // used for printData()
    private Vector currentLongestLine;

    private final Pixy2 pixy;

    /**
     * Constructs a new <code>PixyCam</code> over SPI using Port.kOnboardCS0.
     * @see PixyCam#createInstance(Link, int)
     * @see io.github.pseudoresonance.pixy2api.links.SPILink
     * @see edu.wpi.first.wpilibj.SPI.Port
     */
    public PixyCam() {
        this(new SPILink(), SPI.Port.kOnboardCS0.value);
    }

    /**
     * Constructs a new <code>PixyCam</code>.
     * @param link the communication link to Pixy
     * @param arg the init arg for the link
     * @see io.github.pseudoresonance.pixy2api.links.Link
     * @see io.github.pseudoresonance.pixy2api.Pixy2#init(int)
     */
    public PixyCam(Link link, int arg) {
        pixy = Pixy2.createInstance(link);
        pixy.init(arg);
    }

    /**
     * Turns Pixy's LEDs on or off.
     * @param on <code>true</code> for on, <code>false</code> for off
     * @see io.github.pseudoresonance.pixy2api.Pixy2#setLamp
     */
    public void setLamp(boolean on) {
        byte valueUpperAndLower = on ? (byte) 1 : (byte) 0;
        pixy.setLamp(valueUpperAndLower, valueUpperAndLower);
    }
    
    /**
     * Gets the gaffer tape from Pixy, or <code>null</code> if the tape is not present.
     * @return the gaffer tape from Pixy, or <code>null</code> if the tape is not present
     * @see Tape
     * @see PixyCam#getLongestLine()
     */
    public synchronized Tape getTape() {
        Vector line = this.getLongestLine();
        if(line == null) {
            return null;
        }

        int dX = Math.abs(line.getX0() - line.getX1());
        int dY = Math.abs(line.getY0() - line.getY1());
        double lineAngle;

        if(dY == 0) {
            lineAngle = 90;
        } else {
            double lineAngleFromPixy = Math.toDegrees(Math.atan(dX / dY));

            // trying 0.0099x^2 - 0.33x for now
            lineAngle = 0.0099*Math.pow(lineAngleFromPixy, 2) - 0.33*lineAngleFromPixy;
        }

        return new Tape(lineAngle);
    }
    
    /**
     * Gets the longest line from Pixy, or <code>null</code> if no lines are present.
     * @return the longest line from Pixy as a <code>Vector</code>, or <code>null</code> if no lines are present
     * @see io.github.pseudoresonance.pixy2api.Pixy2Line#getFeatures
     * @see io.github.pseudoresonance.pixy2api.Pixy2Line.Vector
     */
    public synchronized Vector getLongestLine() {
        pixy.getLine().getFeatures(Pixy2Line.LINE_GET_MAIN_FEATURES, Pixy2Line.LINE_VECTOR, true);
        Vector[] lines = pixy.getLine().getVectors();
        currentNumLines = 0;
        if(lines != null)
            currentNumLines = lines.length;

        Vector line = null;
        if(currentNumLines == 1) {
            line = lines[0];
        }
        else if(currentNumLines > 1) {
            Logger.warning("Pixy detected multiple (" + currentNumLines + ") lines!");
            Logger.debug("Detected Lines: " + Arrays.toString(lines));

            // start assuming first line is the longest
            int indexOfLongestLine = 0;
            double lengthOfLongestLine = Math.hypot(lines[0].getX0() - lines[0].getX1(), lines[0].getY0() - lines[0].getY1());

            for(int i = 1; i < currentNumLines; i++) {
                // check if this line is longer
                double length = Math.hypot(lines[i].getX0() - lines[i].getX1(), lines[i].getY0() - lines[i].getY1());
                if(length > lengthOfLongestLine) {
                    lengthOfLongestLine = length;
                    indexOfLongestLine = i;
                }
            }
            
            line = lines[indexOfLongestLine];
        }

        currentLongestLine = line;

        // longest vector
        return line;
    }

    public void getRealLine(Vector line) {
        double y0 = CAMERA_HEIGHT * Math.tan(Math.toRadians(PIXY_LENS + (VERT_FOV/2)-(line.getY0()*VERT_D_PER_P)));
        double x0 = Math.tan(Math.toRadians(line.getX0()*HORIZ_D_PER_P) - (HORIZ_FOV/2)) * CAMERA_HEIGHT * (1/Math.cos(Math.toRadians(PIXY_LENS+(VERT_FOV/2)-(line.getY0()*VERT_D_PER_P))));
        double y1 = CAMERA_HEIGHT * Math.tan(Math.toRadians(PIXY_LENS + (VERT_FOV/2)-(line.getY1()*VERT_D_PER_P)));
        double x1 = Math.tan(Math.toRadians(line.getX1()*HORIZ_D_PER_P) - (HORIZ_FOV/2)) * CAMERA_HEIGHT * (1/Math.cos(Math.toRadians(PIXY_LENS+(VERT_FOV/2)-(line.getY1()*VERT_D_PER_P))));

        double dx = x1 - x0;
        double dy = y1 - y0;

        double angle = Math.toDegrees(Math.atan2(dy, dx));
        double len = Math.hypot(dx, dy);

        Logger.debug("{" + line.getX0() + ", " + line.getY0() + "}, {" + line.getX1() + ", " + line.getY1() + "}");
        Logger.debug("Angle: " + angle);
        Logger.debug("Length: " + len);

        lineAngle = angle;
        lineLength = len;
    } 

    public double getLineAngle()
    {
        return lineAngle;
    }
    public double getLineLength()
    {
        return lineLength;
    }

    public synchronized int getNumLines() {
        return currentNumLines;
    }

    /**
	 * Prints all current Pixy2 tape/line data.
	 */
    public synchronized void printData() {
        Tape tape = this.getTape();

        System.out.println("-------------------------------------------");
        if(tape != null) {

            System.out.println("Tape: " + tape);
            System.out.println("Line: " + currentLongestLine);
            getRealLine(currentLongestLine);

        } else {
            System.out.println("Pixy doesn't see any lines.");
        }
    }

    /**
     * Class to encapsulate information about the gaffer tape.
     */
    public class Tape {
        /** The angle of the gaffer tape in degrees. */
        public final double angle;

        /**
         * Constructs a new <code>Tape</code> object.
         * @param angle the angle of the gaffer tape in degrees
         */
        private Tape(double angle) {
            this.angle = angle;
        }

        /**
		 * <p>Representation: <code>angle</code> degrees</p>
		 * {@inheritDoc}
		 */
        @Override
        public String toString() {
            return angle + " degrees";
        }
    }
}
