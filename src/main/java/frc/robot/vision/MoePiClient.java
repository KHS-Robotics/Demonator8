package frc.robot.vision;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.util.ArrayList;

import frc.robot.subsystems.TankDrive;

/**
 * <p><a href="https://github.com/KHS-Robotics/MoePi">MoePi</a> UDP Client to get 
 * retroreflective tape data for Destination: Deep Space.</p>
 */
public class MoePiClient implements Runnable {
	/** Default UDP Port for MoePi. */
	public static final int DEFAULT_UDP_PORT = 5810;

	/** Offset angle for target since camera is on robot's side. */
	public static final double CAMERA_ANGLE_OFFSET = 10;//TODO: check this on field

	/** x-resolution of the camera */
	public static final double RESOLUTION_X = 640;
	/** y-resolution of the camera */
	public static final double RESOLUTION_Y = 480;
	/** Diagonal Field of View for the Camera. */
    public static final double DIAGONAL_FIELD_OF_VIEW = 68.5;
    /** Horizontal Field of View for the Camera. */
	public static final double HORIZONTAL_FIELD_OF_VIEW = 57.5;
	
    /** Width between the two target's x-coordinates in inches. */
	public static final double TARGET_WIDTH = 10.0; // TODO: find this value
	
	// MoePi status codes
	public static final short STATUS_NONE_FOUND = (short) 1;
	public static final short STATUS_ONE_FOUND = (short) 2;
	public static final short STATUS_TWO_FOUND = (short) 3;
	public static final short STATUS_THREE_FOUND = (short) 4;
	public static final short STATUS_FOUR_FOUND = (short) 5;
	public static final short STATUS_FIVE_FOUND = (short) 6;
	public static final short STATUS_SIX_FOUND = (short) 7;
	public static final short STATUS_ERROR = (short) 0x8000;
	public static final short STATUS_HELLO = (short) 0x8001;
	public static final short STATUS_GOODBYE = (short) 0x8002;
	
	/** The size of the buffer for MoePi, in bytes. */
	public static final int BUFFER_SIZE = 246;

	private double currentRobotHeading, lastRobotHeading;

	private DeepSpaceVisionTarget centerTarget;

	private int lastID;
	private DatagramSocket socket;
	private ArrayList<Box> boxVector;
	private ArrayList<DeepSpaceVisionTarget> targetVector;

	public final String Name;
	public final int Port;
	private final TankDrive Drive;

	/**
	 * Constructs a new <code>MoePiClient</code>.
	 * @throws SocketException if the socket could not be opened, 
	 * or the socket could not bind to the specified local port
	 */
	public MoePiClient(TankDrive drive) throws SocketException {
		this("MoePi", DEFAULT_UDP_PORT, drive);
	}

	/**
	 * Constructs a new <code>MoePiClient</code>.
	 * @param name the name of this specific <code>MoePiClient</code>
	 * @param port the port to retrieve UDP data from
	 * @throws SocketException if the socket could not be opened, 
	 * or the socket could not bind to the specified local port
	 */
	public MoePiClient(String name, int port, TankDrive drive) throws SocketException {
		this.Name = name;
		this.Port = port;
		this.Drive = drive;

		socket = new DatagramSocket(port);
		boxVector = new ArrayList<Box>();
		targetVector = new ArrayList<>();
		new Thread(this).start();
	}

	/**
	 * <p>Continuously updates MoePi telemetry.</p>
	 * {@inheritDoc}
	 */
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
				
				if(status == STATUS_HELLO) {
					lastID = id;
					continue;
				}

				if(id <= lastID) {
					continue;
				}
				lastID = id;

				final int boxAmnt = status - 1;

				synchronized(this) {
					boxVector.clear();
					for(int n = 0; n < boxAmnt; n++) {
						boxVector.add(readBox(buf));
					}

					lastRobotHeading = currentRobotHeading;
					currentRobotHeading = Drive.getHeading();

					this.updateCenterTarget();
				}
			} catch (Exception ex) {
                ex.printStackTrace();
				break;
			}
		}

		socket.close();
	}
	
	/**
	 * Reads a box from the <code>ByteBuffer</code>.
	 * @param buf the <code>ByteBuffer</code> to read from
	 * @return the next box in the <code>ByteBuffer</code>
	 */
	private Box readBox(ByteBuffer buf) {
		double x = buf.getDouble();
		double y = buf.getDouble();
		double w = buf.getDouble();
		double h = buf.getDouble();
		double type = buf.getDouble();

		return new Box((x + w / 2), (y + h / 2), w, h, type);
	}

	public double getLastRobotHeading() {
		return lastRobotHeading;
	}

	public double getRobotHeading() {
		return currentRobotHeading;
	}

	/**
	 * Gets the angle offset of the most centered <code>DeepSpaceVisionTarget</code>
	 * and adds <code>offset</code> to it.
	 * @param offset the amount to add to {@link MoePiClient#getAngle()} 
	 * @return the sum of {@link MoePiClient#getAngle()} and <code>offset</code>
	 * @see MoePiClient#getAngle()
	 */
	public synchronized double getAngle(double offset) {
		return this.getAngle() + offset;
	}

	/**
	 * Gets the angle offset of the most centered <code>DeepSpaceVisionTarget</code>.
	 * @return the angle offset of the most centered <code>DeepSpaceVisionTarget</code>
	 * @see MoePiClient#updateCenterTarget()
	 * @see MoePiClient#getAngle(double)
	 */
	public synchronized double getAngle() {
		try {
			if(centerTarget == null) {
				return 0;
			}
			
			return HORIZONTAL_FIELD_OF_VIEW*(centerTarget.x - 0.5);
		} catch(Exception ex) {
            ex.printStackTrace();
			return 0;
		}
	}

	/**
	 * Gets the distance of the most centered <code>DeepSpaceVisionTarget</code> in inches.
	 * @return the distance of the most centered <code>DeepSpaceVisionTarget</code> in inches
	 * @see MoePiClient#updateCenterTarget()
	 */
	public synchronized double getDistance() {
		try {
			if(centerTarget == null || (centerTarget != null && centerTarget.hasOnlyRight())) {
				return 0;
			}

			double theta = HORIZONTAL_FIELD_OF_VIEW*Math.abs(centerTarget.left.x - centerTarget.right.x);

			// splitting triangle in half to make a right triangle means:
			// tan(theta/2) = width/2 / distance
			// distance = width / 2*tan(theta/2)
			return TARGET_WIDTH / (2 * Math.tan(Math.toRadians(theta / 2.0)));
		} catch(Exception ex) {
            ex.printStackTrace();
			return 0;
		}
	}

	/**
	 * Updates the most centered <code>DeepSpaceVisionTarget</code>.
	 * @see DeepSpaceVisionTarget
	 * @see MoePiClient#updateDeepSpaceTargets()
	 */
	private synchronized void updateCenterTarget() {
		this.updateDeepSpaceTargets();

		if(targetVector.isEmpty()) {
			centerTarget = null;
		}
		else if(targetVector.size() == 1) {
			centerTarget = targetVector.get(0);
		}
		else {
			// start assuming first target is most centered
			int centerBoxIndex = 0;
			DeepSpaceVisionTarget closestCenterTarget = targetVector.get(centerBoxIndex);
			double closestCenterXDiff = Math.abs(closestCenterTarget.x - 0.5);

			for(int i = 1; i < targetVector.size(); i++) {
				// check if this target is closer to the center
				double boxCenterXDiff = Math.abs(targetVector.get(i).x - 0.5);
				if(boxCenterXDiff < closestCenterXDiff) {
					centerBoxIndex = i;
					closestCenterTarget = targetVector.get(i);
					closestCenterXDiff = boxCenterXDiff;
				}
			}

			centerTarget = closestCenterTarget;
		}
	}

	/**
	 * Updates all currently visible Deep Space vision targets.
	 * @see DeepSpaceVisionTarget
	 * @see MoePiClient#updateCenterTarget()
	 */
	private synchronized void updateDeepSpaceTargets() {
		targetVector.clear();

		if(boxVector.size() == 1) {
			if(boxVector.get(0).type == Box.TargetType.RIGHT.value) {
				targetVector.add(new DeepSpaceVisionTarget(null, boxVector.get(0)));
			}
		}
		else if(boxVector.size() >= 2) {
			// tbh not best way to do this/could be faster but max
			// boxVector size is only six anyway
			Box lastBox = boxVector.get(0);
			for(int i = 1; i < boxVector.size(); i++) {
				if(lastBox.type == Box.TargetType.LEFT.value && boxVector.get(i).type == Box.TargetType.RIGHT.value) {
					targetVector.add(new DeepSpaceVisionTarget(lastBox, boxVector.get(i)));
				}
				
				lastBox = boxVector.get(i);
			}
		}
	}

	/**
	 * Prints all current MoePi telemetry.
	 */
	public synchronized void printData() {
		try {
			this.updateDeepSpaceTargets();
			this.updateCenterTarget();

			System.out.println("-------------------------------------------");
			System.out.println("Found Boxes: " + !boxVector.isEmpty());

			for(int i = 0; i < boxVector.size(); i++) {
				System.out.println("Box " + (i+1) + ": " + boxVector.get(i));
			}
			
			System.out.println("Targets: " + targetVector);
			System.out.println("Center Target: " + centerTarget);
			System.out.println("Angle: " + this.getAngle(CAMERA_ANGLE_OFFSET) + " degrees");
			System.out.println("Distance: " + this.getDistance() + " inches");
		} catch(Exception ex) {
            ex.printStackTrace();
		}
	}
	
	/**
	 * Box for MoePi UDP Client.
	 * @see MoePiClient#readBox(ByteBuffer)
	 */
	public static class Box {
		public final double x, y, w, h, area, type;

		/**
		 * Constructs a new instance of <code>Box</code>.
		 * @param x the center x coordinate
		 * @param y the center y coordinate
		 * @param w the width of the box
		 * @param h the height of the box
		 * @param type the value of the <code>TargetType</code> of the box
		 * @see TargetType
		 */
		private Box(double x, double y, double w, double h, double type) {
			this.x = x;
			this.y = y;
			this.w = w;
			this.h = h;
			this.area = w * h;
			this.type = type;
		}

		/**
		 * <p>Representation: <code>type</code> <code>width</code>x<code>height</code> :: (<code>x</code>,<code>y</code>)</p>
		 * {@inheritDoc}
		 */
		@Override
		public String toString() {
			return typeToString(type) + " " + w + "x" + h + " :: (" + x + ", " + y + ")";
		}

		/**
		 * Returns <code>type</code> as a <code>String</code>.
		 * @param type the type as a <code>double</code>
		 * @return <code>type</code> as a <code>String</code>
		 */
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

		/**
		 * Target Type for Destination Deep Space 2019.
		 */
		public enum TargetType {
			NONE(1.0d), LEFT(2.0d), RIGHT(3.0d);

			public final double value;

			private TargetType(double value) {
				this.value = value;
			}
		}
	}

	/**
	 * Vision Target for Destination Deep Space 2019.
	 * @see Box
	 * @see MoePiClient#updateDeepSpaceTargets()
	 * @see MoePiClient#updateCenterTarget()
	 */
	public class DeepSpaceVisionTarget {
		public final double x, y;

		public final Box left;
		public final Box right;

		/**
		 * Constructs a new <code>DeepSpaceVisionTarget</code>.
		 * @param left the left target
		 * @param right the right target
		 */
		private DeepSpaceVisionTarget(Box left, Box right) {
			this.left = left;
			this.right = right;

			if(left != null && right != null) {
				this.x = (left.x + right.x) / 2.0;
				this.y = (left.y + right.y) / 2.0;
			}
			else if(right != null) {
				this.x = right.x;
				this.y = right.y;
			}
			else {
				this.x = this.y = 0;
			}
		}

		/**
		 * Returns if there is only a right target to the pair. The purpose of this is the fact that our 
		 * vision camera is mounted on the right side of the elevator (still in a fixed position, though) 
		 * facing forward (meaning it's not centered). This means as we approach a target to score we (should) 
		 * only see the right target (however, this is not until we are really, really close).
		 * @return <code>true</code> if there is only a right target to the pair, 
		 * <code>false</code> if there is both a left/right target to the pair
		 */
		public boolean hasOnlyRight() {
			return left == null;
		}

		/**
		 * <p>Representation: (x, y) [{<code>left.toString()</code>}, {<code>right.toString()</code>}]</p>
		 * {@inheritDoc}
		 */
		@Override
		public String toString() {
			return "(" + x + ", " + y + ") [{ " + left + " }, { " + right + " }]";
		}
	}
}
