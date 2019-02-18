/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2Line.Vector;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public class PixyCam {
    private Pixy2 pixy;
    private SPILink link = new SPILink();

    public final int CAMERA_MAX_X = 78;
    public final int CAMERA_MAX_Y = 51;

    public PixyCam() {
        link = new SPILink();
        link.open(1);
        pixy = Pixy2.createInstance(link);
        pixy.init();
    }
    
    public void getLines() {
    //System.out.println(pixy.getLine().getAllFeatures());
        Vector[] vec = pixy.getLine().getVectors();
        System.out.println("*************************************************************************************");
        // System.out.println(vec == null);
        if(vec != null) {
            for(Vector v : vec) {
                System.out.println(v);
                System.out.println(getAngle(v));
            }
        }
        

        System.out.println("*************************************************************************************");
        
    }

    public static double getAngle(Vector vec) {
        int deltaX = Math.abs(vec.getX0() - vec.getX1())- (78 / 2);
        int deltaY = Math.abs(vec.getY0() - vec.getY1());
        double angle;
        
        if(deltaX != 0) {
            angle = Math.atan(deltaY / deltaX);
        }
        else {
            angle = Math.PI / 2;
        }

        return angle;
    }

    public void lampOn() {
        pixy.setLamp((byte) 1, (byte) 1);
    }

    public void lampOff() {
        pixy.setLamp((byte) 0, (byte) 0);
    }

}