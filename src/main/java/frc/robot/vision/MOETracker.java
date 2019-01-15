package frc.robot.vision;

public interface MOETracker extends Runnable {
	//virtual screen dimensions
	public static final double pixelsWide = 640, pixelsHigh = 480;
	/**
	 * @return the center of main target with coordinates as screen percentages ranged [0,1]
	 */
	public double[] getCenter();
}
