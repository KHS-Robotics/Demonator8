package frc.robot.vision;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.util.Vector;

public class UDPTracker implements MOETracker {
	static final short noneFound = 1, oneFound = 2, twoFound = 3;
	protected int lastID;
	protected DatagramSocket socket;
	protected String name;
	protected Vector<Box> boxVector;

	public UDPTracker(String name, final int port) throws SocketException {
		this.name = name;
		socket = new DatagramSocket(port);
		boxVector = new Vector<Box>();
		new Thread(this).start();
	}
	public static UDPTracker getTracker(String name, final int port){
		try{
			return new UDPTracker(name, port);
		}catch (SocketException e){
			e.printStackTrace();
			return null;
		}
	}
	@Override
	public void run() {
		ByteBuffer buf = ByteBuffer.allocate(72);
		DatagramPacket packet = new DatagramPacket(buf.array(), buf.limit());
		while (true) {
			try {
				buf.position(0);
				packet.setLength(buf.limit());
				socket.receive(packet);
				final int id = buf.getInt();
				if (id <= lastID)
					continue;
				lastID = id;
				final short status = buf.getShort();
				int boxAmnt = status - 1;
				synchronized (this) {
					boxVector.clear();
					for(int n=0;n<boxAmnt;n++)
						boxVector.add(readBox(buf));
				}
			} catch (Exception e) {
				e.printStackTrace();
				break;
			}
		}
		socket.close();
	}
	private Box readBox(ByteBuffer buf){
		double x = buf.getDouble();
		double y = buf.getDouble();
		double w = buf.getDouble();
		double h = buf.getDouble();
		return new Box((x + w / 2), (y + h / 2), w*h, w, h);
	}
	public synchronized double[] getCenter(){
		try{
			Box box;
			synchronized (this) {
				box = boxVector.get(0);
				return new double[]{box.x, box.y};
			}
		}catch(Exception e){
			e.printStackTrace();
			return new double[]{-1,-1};
		}
	}

	public Vector<Box> getBoxes() {
		return boxVector;
	}

	public class Box{
		double x,y,area,w,h;
		public Box(double x, double y, double area, double w, double h){
			this.x=x;
			this.y=y;
			this.area=area;
			this.w=w;
			this.h=h;
		}

		@Override
		public String toString() {
			return w + "x" + h + " :: (" + x + ", " + y + ")";
		}
	}
}