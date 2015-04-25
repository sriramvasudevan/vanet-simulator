import java.util.LinkedList;
import java.util.Queue;

/**
 * The class that provides the template for the Vehicle object
 * 
 * @author sriram
 *
 */
public class Vehicle {
	/**
	 * Velocity of the vehicle
	 */
	double velocity;
	/**
	 * Vehicle position (X coordinate)
	 */
	float pos_x;
	/**
	 * Vehicle position (Y coordinate)
	 */
	float pos_y;
	/**
	 * Countdown till velocity change
	 */
	int cntdown;
	/**
	 * Vehicle ID
	 */
	int id;
	/**
	 * Vehicle position in terms of segments (X coordinate)
	 */
	int segment_x;
	/**
	 * Vehicle position in terms of segments (Y coordinate)
	 */
	int segment_y;
	/**
	 * True if vehicle is currently transmitting
	 */
	boolean tx;
	/**
	 * ID of vehicle from which this vehicle is currently receiving (V2V)
	 */
	int tx_id;
	/**
	 * ID of packet being received currently
	 */
	int pkt_id;
	/**
	 * Motion is L to R (+1) or R to L (-1)
	 */
	int x_dxn;
	/**
	 * Motion is T to B (+1) or B to T (-1)
	 */
	int y_dxn;
	/**
	 * Packet queue of the vehicle
	 */
	Queue<Packet> pktqueue;
	/**
	 * Backoff counter. Samples from a contention window of size min_cw to max_cw
	 */
	int backoff;
	/**
	 * Time left to finish receiving packet. Packet reception is ongoing.
	 */
	int recv_tx;
	/**
	 * Class variable; counter variable
	 */
	private static int counter = 0;
	
	/**
	 * Public class constructor for a new vehicle object. Takes some parameters,
	 * sets other variables by itself.
	 * 
	 * @param velocity
	 *            The initial velocity of the vehicle
	 * @param x_dxn
	 *            whether motion is from L to R or vice versa
	 * @param y_dxn
	 *            whether motion is from T to B or vice versa
	 * @param pos_x
	 *            X coordinate of vehicle
	 * @param pos_y
	 *            Y coordinate of vehicle
	 */
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

	/**
	 * A timer that samples from an exponential distribution and counts down to
	 * zero, upon which it resets. Used to time velocity changes for vehicles.
	 * 
	 * @return A boolean that tells if the counter has reached zero or not.
	 */
	public boolean checkTimer() {
		this.cntdown--;
		if (cntdown == 0) {
			this.cntdown = (int) Simulator
					.generateExp(Simulator.lambda_countdown);
			return true;
		}
		return false;
	}

	/**
	 * Public static method to reset the unique vehicle ID generator.
	 */
	public static void resetCounter() {
		Vehicle.counter = 0;
	}
}
