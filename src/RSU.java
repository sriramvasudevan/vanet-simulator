public class RSU {
	int id;
	int segment_x;
	int segment_y;
	boolean on;
	boolean solar;
	int idle;
	int recv_tx;
	int tx_id;
	int pkt_id;
	double battery;
	private static int counter = 0;

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
	
	public static void resetCounter() {
		RSU.counter = 0;
	}
}
