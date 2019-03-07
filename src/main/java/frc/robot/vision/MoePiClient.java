/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.util.ArrayList;

import frc.robot.logging.Logger;
import frc.robot.subsystems.TankDrive;

/**
 * <p><a href="https://github.com/KHS-Robotics/MoePi">MoePi</a> UDP Client to get 
 * retroreflective tape data for Destination: Deep Space.</p>
 */
public class MoePiClient implements Runnable {
	/** Default UDP Port for MoePi. */
	public static final int DEFAULT_UDP_PORT = 5810;

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

	private int lastID;
	private DatagramSocket socket;
	private ArrayList<Box> boxVector;
	private double currentRobotHeading, lastRobotHeading;

	public final TankDrive Drive;
	public final String Name;
	public final int Port;

	/**
	 * Constructs a new <code>MoePiClient</code>.
	 * @param drive the drive train
	 * @throws SocketException if the socket could not be opened, 
	 * or the socket could not bind to the specified local port
	 */
	public MoePiClient(TankDrive drive) throws SocketException {
		this(drive, "MoePi", DEFAULT_UDP_PORT);
	}

	/**
	 * Constructs a new <code>MoePiClient</code>.
	 * @param drive the drive train
	 * @param name the name of this specific <code>MoePiClient</code>
	 * @param port the port to retrieve UDP data from
	 * @throws SocketException if the socket could not be opened, 
	 * or the socket could not bind to the specified local port
	 */
	public MoePiClient(TankDrive drive, String name, int port) throws SocketException {
		this.Drive = drive;
		this.Name = name;
		this.Port = port;

		socket = new DatagramSocket(port);
		boxVector = new ArrayList<Box>();

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

		SmartDashboard.putBoolean(this.Name, true);

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
					currentRobotHeading = this.Drive.getHeading();
				}
			} catch (Exception ex) {
				Logger.error(this.Name + " UDP Client crashed!", ex);
				break;
			}
		}

		SmartDashboard.putBoolean(this.Name, false);

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

	/**
	 * Gets the found boxes from MoePi, ordered from left to right (smallest x-value first).
	 * @return the found boxes from MoePi
	 * @see Box
	 */
	public ArrayList<Box> getBoxes() {
		return boxVector;
	}

	/**
	 * Returns all currently visible Deep Space vision targets in an <code>ArrayList</code>.
	 * @return all currently visible Deep Space vision targets in an <code>ArrayList</code>
	 * @see DeepSpaceVisionTarget
	 * @see MoePiClient#getVisionTarget()
	 */
	public synchronized ArrayList<DeepSpaceVisionTarget> getVisionTargets() {
		try {
			synchronized(this) {
				ArrayList<DeepSpaceVisionTarget> targets = new ArrayList<>();

				if(boxVector.size() == 1) {
					if(boxVector.get(0).type == Box.TargetType.RIGHT.value) {
						targets.add(new DeepSpaceVisionTarget(null, boxVector.get(0)));
					}
				}
				else if(boxVector.size() >= 2) {
					// tbh not best way to do this/could be faster but max
					// boxVector size is only six anyway
					Box lastBox = boxVector.get(0);
					for(int i = 1; i < boxVector.size(); i++) {
						if(lastBox.type == Box.TargetType.LEFT.value && boxVector.get(i).type == Box.TargetType.RIGHT.value) {
							targets.add(new DeepSpaceVisionTarget(lastBox, boxVector.get(i)));
						}
						
						lastBox = boxVector.get(i);
					}
				}
				
				return targets;
			}
		} catch(Exception ex) {
			Logger.error("Failed to getVisionTarget!", ex);
			return new ArrayList<DeepSpaceVisionTarget>(0);
		}
	}

	/**
	 * Returns the most centered <code>DeepSpaceVisionTarget</code>.
	 * @return the most centered <code>DeepSpaceVisionTarget</code>
	 * @see DeepSpaceVisionTarget
	 * @see MoePiClient#getVisionTargets()
	 */
	public synchronized DeepSpaceVisionTarget getVisionTarget() {
		try {
			synchronized(this) {
				ArrayList<DeepSpaceVisionTarget> targets = this.getVisionTargets();
				DeepSpaceVisionTarget target;

				if(targets.isEmpty()) {
					target = null;
				}
				else if(targets.size() == 1) {
					target = targets.get(0);
				}
				else {
					// start assuming first target is most centered
					int centerBoxIndex = 0;
					DeepSpaceVisionTarget centerTarget = targets.get(centerBoxIndex);
					double closestCenterXDiff = Math.abs(centerTarget.x - 0.5);

					for(int i = 1; i < targets.size(); i++) {
						// check if this target is closer to the center
						double boxCenterXDiff = Math.abs(targets.get(i).x - 0.5);
						if(boxCenterXDiff < closestCenterXDiff) {
							centerBoxIndex = i;
							centerTarget = targets.get(i);
							closestCenterXDiff = boxCenterXDiff;
						}
					}

					target = centerTarget;
				}

				return target;
 			}
		} catch(Exception ex) {
			Logger.error("Failed to getVisionTarget!", ex);
			return null;
		}
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
	 * @see MoePiClient#getVisionTarget()
	 * @see MoePiClient#getAngle(double)
	 */
	public synchronized double getAngle() {
		try {
			synchronized(this) {
				DeepSpaceVisionTarget target = this.getVisionTarget();
				if(target == null) {
					return 0;
				}
				
				return HORIZONTAL_FIELD_OF_VIEW*(target.x - 0.5);
			}
		} catch(Exception ex) {
			Logger.error("Failed to getAngle!", ex);
			return 0;
		}
	}

	/**
	 * Gets the distance of the most centered <code>DeepSpaceVisionTarget</code> in inches.
	 * @return the distance of the most centered <code>DeepSpaceVisionTarget</code> in inches
	 * @see MoePiClient#getVisionTarget()
	 */
	public synchronized double getDistance() {
		try {
			synchronized(this) {
				DeepSpaceVisionTarget target = this.getVisionTarget();
				if(target == null || (target != null && target.hasOnlyRight())) {
					return 0;
				}

				double theta = HORIZONTAL_FIELD_OF_VIEW*Math.abs(target.left.x - target.right.x);

				// splitting triangle in half to make a right triangle means:
				// tan(theta/2) = width/2 / distance
				// distance = width / 2*tan(theta/2)
				return TARGET_WIDTH / (2 * Math.tan(Math.toRadians(theta / 2.0)));
			}
		} catch(Exception ex) {
			Logger.error("Failed to getDistance!", ex);
			return 0;
		}
	}

	/**
	 * Gets the current robot heading in degrees.
	 * @return the current robot heading in degrees
	 */
	public double getRobotHeading() {
		return currentRobotHeading;
	}

	/**
	 * Gets the last (previous iteration's) robot heading in degrees.
	 * @return the last (previous iteration's) robot heading in degrees
	 */
	public double getLastRobotHeading() {
		return lastRobotHeading;
	}

	/**
	 * Prints all current MoePi telemetry.
	 */
	public synchronized void printData() {
		try {
			synchronized(this) {
				ArrayList<Box> boxes = this.getBoxes();

				System.out.println("-------------------------------------------");
				System.out.println("Found Boxes: " + !boxes.isEmpty());

				for(int i = 0; i < boxes.size(); i++) {
					System.out.println("Box " + (i+1) + ": " + boxes.get(i));
				}
				
				System.out.println("Targets: " + this.getVisionTargets());
				System.out.println("Center Target: " + this.getVisionTarget());
				System.out.println("Angle: " + this.getAngle() + " degrees");
				System.out.println("Distance: " + this.getDistance() + " inches");
			}
		} catch(Exception ex) {
			Logger.error("Failed to printData!", ex);
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
	 * @see MoePiClient#getVisionTargets()
	 * @see MoePiClient#getVisionTarget()
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