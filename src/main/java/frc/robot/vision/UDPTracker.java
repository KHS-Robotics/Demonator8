package frc.robot.vision;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;

import frc.robot.subsystems.TankDrive;

/**
 * <p>MoePi UDP Client</p>
 * https://github.com/KHS-Robotics/MoePi
 */
public class UDPTracker implements Runnable {
	public static final int MOEPI_UDP_PORT = 5810;
	// virtual screen dimensions
    public static final double PIXEL_WIDTH = 640, PIXEL_HEIGHT = 480;
    // Field of View
    public static final double FIELD_OF_VIEW = 57.5;
    // Width between target's x-coordinates
	public static final double TARGET_WIDTH = 10.0; // TODO: find this value
	
	public static final short NONE_FOUND = (short) 1;
	public static final short ONE_FOUND = (short) 2;
	public static final short TWO_FOUND = (short) 3;
	public static final short THREE_FOUND = (short) 4;
	public static final short FOUR_FOUND = (short) 5;
	public static final short FIVE_FOUND = (short) 6;
	public static final short SIX_FOUND = (short) 7;
	public static final short HELLO_WORLD = (short) 0x8001;
	
	protected int lastID;

	public static final int BUFFER_SIZE = 246;
	private DatagramSocket socket;
	private ArrayList<Box> boxVector;

	private double currentRobotHeading, lastRobotHeading;

	public final TankDrive drive;
	public final String name;
	public final int port;

	public UDPTracker(TankDrive drive, String name, int port) throws SocketException {
		this.drive = drive;
		this.name = name;
		this.port = port;
		socket = new DatagramSocket(port);
		boxVector = new ArrayList<Box>();
		new Thread(this).start();
	}

	public UDPTracker(TankDrive drive) throws SocketException {
		this(drive, "MoePi", MOEPI_UDP_PORT);
	}

	@Override
	public void run() {
		ByteBuffer buf = ByteBuffer.allocate(BUFFER_SIZE);
		DatagramPacket packet = new DatagramPacket(buf.array(), buf.limit());
		
		while(true) {
			try {
				buf.position(0);
				packet.setLength(buf.limit());
				socket.receive(packet);
				
				final int id = buf.getInt();
				final int status = buf.getShort();
				
				if(status == HELLO_WORLD) {
					lastID = id;
					continue;
				}

				if(id <= lastID) {
					continue;
				}
				lastID = id;

				final int boxAmnt = status - 1;

				synchronized (this) {
					boxVector.clear();
					for(int n = 0; n < boxAmnt; n++) {
						boxVector.add(readBox(buf));
					}

					lastRobotHeading = currentRobotHeading;
					currentRobotHeading = this.drive.getHeading();
				}
			} catch (Exception ex) {
				ex.printStackTrace();
				break;
			}
		}

		socket.close();
	}
	
	private Box readBox(ByteBuffer buf) {
		double x = buf.getDouble();
		double y = buf.getDouble();
		double w = buf.getDouble();
		double h = buf.getDouble();
		double type = buf.getDouble();

		return new Box((x + w / 2), (y + h / 2), w, h, type);
	}

	public ArrayList<Box> getBoxes() {
		return boxVector;
	}

	// TODO: implement logic for potentially six targets and left/right
	public synchronized double[] getCenter() {
		try {
			synchronized(this) {
				int numBoxes = boxVector.size();

				Box box1, box2;
				if(numBoxes == 0) {
					return null;
				}
				else if(numBoxes == 1) {
					box1 = boxVector.get(0);
					box2 = box1;
				}
				else {
					box1 = boxVector.get(0);
					box2 = boxVector.get(1);
				}
				
				return new double[] {
					(box1.x + box2.x) / 2.0, 
					(box1.y + box2.y) / 2.0
				};
			}
		} catch(Exception ex) {
			ex.printStackTrace();
			return null;
		}
	}

	public synchronized double getAngle() {
		try {
			synchronized(this) {
				double[] center = this.getCenter();
				if(center == null) {
					return 0;
				}

				double centerX = center[0];
				
				return FIELD_OF_VIEW*(centerX - 0.5);
			}
		} catch(Exception ex) {
			ex.printStackTrace();
			return 0;
		}
	}

	// TODO: implement logic for potentially six targets and left/right
	public synchronized double getDistance() {
		try {
			synchronized(this) {
				int numBoxes = boxVector.size();

				Box box1, box2;
				if(numBoxes <= 1) {
					return 0;
				} else {
					box1 = boxVector.get(0);
					box2 = boxVector.get(1);
				}

				double xDiff = Math.abs(box1.x - box2.x);
				double angle = FIELD_OF_VIEW*xDiff;

				// splitting triangle in half to make a right triangle means:
				// tan(theta/2) = width/2 / distance
				// distance = width / 2*tan(theta/2)
				return TARGET_WIDTH / (2 * Math.tan(Math.toRadians(angle / 2.0)));
			}
		} catch(Exception ex) {
			ex.printStackTrace();
			return 0;
		}
	}

	public double getRobotHeading() {
		return currentRobotHeading;
	}

	public double getLastRobotHeading() {
		return lastRobotHeading;
	}

	public synchronized void printData() {
        synchronized(this) {
            try {
				ArrayList<Box> targets = this.getBoxes();

                System.out.println("--------------------------------------");
                System.out.println("Found Targets: " + !targets.isEmpty());
    
                for(int i = 0; i < targets.size(); i++) {
                    System.out.println("Box " + (i+1) + ": " + targets.get(i));
                }
    
                System.out.println("Center: " + Arrays.toString(this.getCenter()));
                System.out.println("Angle: " + this.getAngle() + " degrees");
                System.out.println("Distance: " + this.getDistance() + " inches");
            } catch(Exception ex) {
                ex.printStackTrace();
            }
        }
    }
}
