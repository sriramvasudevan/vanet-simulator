/**
 * The Packet class models packets to be transmitted and received
 * @author sriram
 *
 */
public class Packet implements Cloneable{
	/**
	 * Packet size in bytes
	 */
	int size;
	/**
	 * Packet ID
	 */
	int id;
	/**
	 * Time taken to transmit the packet
	 */
	int tx_time;
	/**
	 * Vehicle of origin (ID)
	 */
	int origin_id;
	/**
	 * Time of creation
	 */
	int create_time;
	/**
	 * Class variable; the counter variable
	 */
	private static int counter = 0;
	/**
	 * Time To Live for the packet. Dropped when this reaches zero
	 */
	int ttl;

	/**
	 * The class constructor
	 * @param origin_id Origin vehicle's ID
	 * @param size size in bytes
	 */
	Packet(int origin_id, int size) {
		this.size = size;
		this.id = Packet.counter;
		this.ttl = Simulator.max_ttl;
		this.origin_id = origin_id;
		this.create_time = Simulator.timestep_no;
		Packet.counter++;
		this.tx_time = (int) (this.size/Simulator.bandwidth);
	}

	/**
	 * A public static method to reset the counter that generates unique packet IDs
	 */
	public static void resetCounter() {
		Packet.counter = 0;
	}
	
	/**
	 * Generates a copy of the packet object that calls this function.
	 */
	@Override
	protected Packet clone() {
        try {
			return (Packet) super.clone();
		} catch (CloneNotSupportedException e) {
			e.printStackTrace();
		}
		return null;
    }
}
