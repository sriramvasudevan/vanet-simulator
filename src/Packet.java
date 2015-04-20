import java.util.ArrayList;

public class Packet implements Cloneable{
	int size; // in kb
	int id;
	int tx_time;
	int origin_id;
	int create_time;
	private static int counter = 0;
	int ttl;
	ArrayList<String> logfile = new ArrayList<String>();

	Packet(int origin_id, int size) {
		this.size = size;
		this.id = Packet.counter;
		this.ttl = Simulator.max_ttl;
		this.origin_id = origin_id;
		this.create_time = Simulator.timestep_no;
		Packet.counter++;
		this.tx_time = (int) (this.size/Simulator.bandwidth);
	}

	public static void resetCounter() {
		Packet.counter = 0;
	}
		
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
