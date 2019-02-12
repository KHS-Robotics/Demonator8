package frc.robot.vision;

/**
 * <p>Box for MoePi UDP Client</p>
 * https://github.com/KHS-Robotics/MoePi
 */
public class Box {
    public final double x, y, w, h, area, type;

    public Box(double x, double y, double w, double h, double type) {
        this.x = x;
        this.y = y;
        this.w = w;
        this.h = h;
        this.area = w * h;
        this.type = type;
    }

    @Override
    public String toString() {
        return typeToString(type) + " " + w + "x" + h + " :: (" + x + ", " + y + ")";
    }

    public String typeToString(double type) {
        if(type == 1) {
            return "NONE";
        }
        if(type == 2) {
            return "LEFT";
        }
        if(type == 3) {
            return "RIGHT";
        }

        return "UNKNOWN TYPE";
    }
}