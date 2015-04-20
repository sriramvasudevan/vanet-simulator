import java.util.LinkedList;
import java.util.Queue;
import java.util.Random;

public class Vehicle {
	double velocity;
	float pos_x;
	float pos_y;
	int cntdown;
	int id;
	int segment_x;
	int segment_y;
	boolean tx;
	int tx_id;
	int pkt_id;
	int x_dxn;
	int y_dxn;
	Queue<Packet> pktqueue;
	int backoff;
	int recv_tx;
	private static int counter = 0;
	static Random rand = new Random();

	Vehicle(double velocity, int x_dxn, int y_dxn, float pos_x, float pos_y) {
		this.velocity = velocity;
		this.cntdown = 1;
		checkTimer();
		this.pktqueue = new LinkedList<Packet>();
		this.id = Vehicle.counter;
		this.pos_x = pos_x;
		this.pos_y = pos_y;
		this.segment_x = (int) (this.pos_x / Simulator.segment_len_x);
		this.segment_y = (int) (this.pos_y / Simulator.segment_len_y);
		this.backoff = 1;
		this.recv_tx = 0;
		this.x_dxn = x_dxn;
		this.y_dxn = y_dxn;
		this.tx = false;
		this.tx_id = -1;
		this.pkt_id = -1;
		Vehicle.counter++;
	}

	public boolean checkTimer() {
		this.cntdown--;
		if (cntdown == 0) {
			this.cntdown = (int) Simulator.generateExp(Simulator.lambda_countdown);
			return true;
		}
		return false;
	}

	public static void resetCounter() {
		Vehicle.counter = 0;
	}
}
