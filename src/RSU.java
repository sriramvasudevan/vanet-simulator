/**
 * The RSU class models an RSU in the VANET simulator
 * 
 * @author sriram
 *
 */
public class RSU {
	/**
	 * RSU ID
	 */
	int id;
	/**
	 * RSU position in terms of segments (X coordinate)
	 */
	int segment_x;
	/**
	 * RSU position in terms of segments (Y coordinate)
	 */
	int segment_y;
	/**
	 * Boolean indicating if the RSU is on.
	 */
	boolean on;
	/**
	 * Boolean indicating if the RSU is using solar power
	 */
	boolean solar;
	/**
	 * -1 if no one is transmitting in the segment where the RSU is located, ID
	 * of transmitting vehicle otherwise. Doesn't mean RSU being idle or not.
	 */
	int idle;
	/**
	 * Time left to receive the packet. Transmission is ongoing.
	 */
	int recv_tx;
	/**
	 * Vehicle transmitting the packet
	 */
	int tx_id;
	/**
	 * ID of the packet being received
	 */
	int pkt_id;
	/**
	 * Current battery level at the RSU.
	 */
	double battery;
	/**
	 * Class variable; counter variable.
	 */
	private static int counter = 0;

	/**
	 * The constructor that initializes all variables for the object, and sets a
	 * unique ID obtained from a counter
	 * 
	 * @param segment_x
	 *            the X coordinate of the RSU
	 * @param segment_y
	 *            the Y coordingate of the RSU
	 */
	RSU(int segment_x, int segment_y) {
		this.id = RSU.counter;
		this.segment_x = segment_x;
		this.segment_y = segment_y;
		this.on = false;
		this.solar = false;
		this.idle = -1;
		this.recv_tx = 0;
		this.tx_id = -1;
		this.pkt_id = -1;
		this.battery = 0.0;
		RSU.counter++;
	}

	/**
	 * Public static method that resets the unique ID generating counter
	 */
	public static void resetCounter() {
		RSU.counter = 0;
	}
}
