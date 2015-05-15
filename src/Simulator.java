import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.Queue;
import java.util.Random;
/*
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartUtilities;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
*/
/**
 * The main simulator class. Handles the entire VANET simulation.
 * 
 * @author sriram
 * 
 */
public class Simulator {
	// Parameters
	/**
	 * No. of timesteps in an episode
	 */
	static int no_timesteps = 10000;
	/**
	 * No. of timesteps in an episode (RPR simulation)
	 */
	static int rpr_no_timesteps = 1000;
	/**
	 * No. of episodes
	 */
	static int no_episodes = 4;
	/**
	 * No. of episodes (RPR simulation)
	 */
	static int rpr_no_episodes = 1;
	/**
	 * Length of the road in the X direction, in meters.
	 */
	static int road_len_x = 20000;
	/**
	 * Length of a segment in the X direction, in meters.
	 */
	static int segment_len_x = 2000;
	/**
	 * Length of the road in the Y direction, in meters. Setting road_len_y =
	 * segment_len_y ensures there is only one L-R road
	 */
	static int road_len_y = 1000;
	/**
	 * Length of a segment in the Y direction, in meters.
	 */
	static int segment_len_y = 1000;
	/**
	 * Size of a packet in bytes
	 */
	static int pkt_size = 200;
	/**
	 * Number of hops to check for idleness. No. of hops to check for hidden
	 * terminals.
	 */
	static int num_hops = 3;
	/**
	 * Contention window lower bound.
	 */
	static int cw_min = 15;
	/**
	 * Contention window upper bound.
	 */
	static int cw_max = 1023;
	/**
	 * Max time to live before pkt is dropped
	 */
	static int max_ttl = 10;
	// Remove T-B roads
	/**
	 * True turns on traffic in the Y direction as well
	 */
	static boolean vert_traffic = false;
	/**
	 * Maximum battery capacity, in Whr.
	 */
	static double max_battery = 240;
	/**
	 * Length of a timeslot, in terms of number of timesteps. Used to keep track
	 * of number of packets delivered in a timeslot.
	 */
	static int timeslot_len = 10;
	/**
	 * Radio power during reception, in J
	 */
	static int rx_pow = 10;
	/**
	 * Radio power during idle state, in J
	 */
	static int res_pow = 9;
	/**
	 * Bandwidth, in Mbps
	 */
	static double bandwidth = 40;
	/**
	 * Battery profile in Whr. Change this for different times of the day
	 */
	static int battery_profile = 40;
	/**
	 * Switching energy, in J
	 */
	static int delta_e_switch = 50;
	/**
	 * Minimum battery value, in Whr
	 */
	static int battery_delta = 12;
	/**
	 * Grid energy cost in $ per kWhr
	 */
	static double grid_cost = 0.06;
	/**
	 * Solar energy cost in $ per KWhr
	 */
	static double solar_cost = 0.001;
	/**
	 * An array to store minimum PDR values (used by RPR). Each element is used
	 * in a different episode, so that trends can be observed if different
	 * values are given.
	 */
	static double[] min_pdr = new double[no_episodes];
	/**
	 * A boolean to set if cars are to be a fixed number (true), or to be
	 * sampled (false).
	 */
	static boolean fixed_cars = false;
	/**
	 * Setting this to true selects all RSUs, giving a uniform distribution.
	 * False enables RPR.
	 */
	static boolean uniform = false;
	/**
	 * Number of fixed cars to generate, if fixed_cars is true. Unused if false.
	 */
	static int fixed_cars_no = 300; // No. of cars to enter from each dxn
	/**
	 * A commandline mode that when set true accepts a single parameter, the
	 * fixed no. of cars, and prints a single value, the average PDR. Useful to
	 * plot number of vehicles vs PDR.
	 */
	static boolean pdr_fixedcars = true;
	/**
	 * Increment the minimum PDR after each episode if true.
	 */
	static boolean inc_min_pdr = true;
	/**
	 * Initial minimum PDR to use
	 */
	static double init_min_pdr = 0.01;
	/**
	 * Step by which to increase the minimum PDR, after every episode.
	 */
	static double step_min_pdr = 0.01;
	/**
	 * Noise constant in dBm
	 */
	static double NOISE_DBM = -127.0;
	/**
	 * Tx Power for Vehicle and RSU in dBm
	 */
	static double TxPower = 33;
	/**
	 * SINR Threshold value
	 */
	static double SINR_THRESH = 0.001;

	// Exponential lambdas
	/**
	 * Parameter for the exponential sampling vehicles
	 */
	static double lambda_vehicle = 3.0;
	/**
	 * Parameter for the exponential sampling velocity
	 */
	static double lambda_velocity = 0.1;
	/**
	 * Parameter for the exponential sampling Packets
	 */
	static double lambda_pkt = 3.0;
	/**
	 * Parameter for the exponential sampling the countdown timer of the
	 * vehicles
	 */
	static double lambda_countdown = 0.2;

	// Logfile Trackers
	/**
	 * Set to true if packets are to be tracked through the simulation. Displays
	 * data if true.
	 */
	static boolean trackPkt = false;
	/**
	 * Set to true if vehicles are to be tracked through the simulation.
	 * Displays data if true.
	 */
	static boolean trackVehicle = false;
	/**
	 * Set to true if RSUs are to be tracked through the simulation. Displays
	 * data if true.
	 */
	static boolean trackRSU = false;
	/**
	 * Print a separator indicating the beginning of a new timestep, if true.
	 */
	static boolean printTimestep = true;
	/**
	 * Print a separator indicating the beginning of a new episode, if true.
	 */
	static boolean printEpisode = true;
	/**
	 * Print final statistics collected after every episode, as well as average
	 * statistics
	 */
	static boolean finalStats = true;
	/**
	 * Print only average PDR in a separate line. Set to true automatically if
	 * pdr_fixedcars is true.
	 */
	static boolean printAvgPDR = false;

	// Output datapts, graphs
	/**
	 * Print vehicle arrival distribution and avg. segment density distribution
	 * details. Useful if plots are to be generated external to the program.
	 */
	static boolean printData = false;
	/**
	 * Plot vehicle distribution and avg. segment density distribution.
	 */
	static boolean genGraphs = true;

	// Simulator vars
	/**
	 * The random variable generator
	 */
	static Random rand = new Random();
	/**
	 * The 2D list containing the RSUs,
	 */
	static ArrayList<ArrayList<RSU>> rsulist = new ArrayList<ArrayList<RSU>>();
	/**
	 * The list containing current vehicles in the grid.
	 */
	static ArrayList<Vehicle> vehlist = new ArrayList<Vehicle>();
	/**
	 * Number of packets received
	 */
	static int recvd = 0;
	/**
	 * Time taken to receive a packet
	 */
	static int recv_time = 0;
	/**
	 * Number of packets generated
	 */
	static int generated = 0;
	/**
	 * Number of packets dropped
	 */
	static int dropped = 0;
	/**
	 * Number of packets delivered at an RSU, in a timeslot
	 */
	static int[][] num_delivered = new int[road_len_x / segment_len_x][road_len_y
			/ segment_len_y];
	/**
	 * Current episode number
	 */
	static int episode_no = 0;
	/**
	 * Current timestep number
	 */
	static int timestep_no = 0;
	/**
	 * Average PDR after all episodes
	 */
	static double avgpdr = 0.0;
	/**
	 * Average packet reception time after all episodes
	 */
	static double avgrxtime = 0.0;
	/**
	 * Operational exchange costs
	 */
	static double opex = 0.0;
	/**
	 * Total energy consumed
	 */
	static double total_energy = 0.0;
	/**
	 * Average OPEX costs, after all episodes
	 */
	static double avgopex = 0.0;
	/**
	 * Average energy consumed, after all episodes
	 */
	static double avgenergy = 0.0;
	/**
	 * 2D array to keep track of avg segment vehicle density distribution
	 */
	static double[] seg_dist = new double[road_len_x / segment_len_x];
	/**
	 * 2D array to keep track of vehicle arrival distribution
	 */
	static double[] veh_dist = new double[no_timesteps];
	/**
	 * Time taken to transmit a single packet, computed using pkt_size and
	 * bandwidth
	 */
	static int tx_delay = (int) (pkt_size / bandwidth);

	/**
	 * The main function. If pdr_fixedcars is true, the fixed no of cars can be
	 * passed as an argument, and will print the average pdr. If pdr_fixedcars
	 * is false, doesn't take any parameter. Print booleans can be modified to
	 * change displayed output.
	 * 
	 * @param args
	 */
	public static void main(String[] args) {
		if (pdr_fixedcars) {
			fixed_cars_no = Integer.parseInt(args[0]);
			SINR_THRESH = Double.parseDouble(args[1]);
			fixed_cars = true;
			uniform = true;
			trackPkt = false;
			trackVehicle = false;
			trackRSU = false;
			printTimestep = false;
			printEpisode = false;
			finalStats = false;
			printAvgPDR = true;
			printData = false;
			genGraphs = false;
		}

		if (inc_min_pdr) {
			min_pdr[0] = init_min_pdr;
			for (int i = 1; i < min_pdr.length; i++) {
				min_pdr[i] = min_pdr[i - 1] + step_min_pdr;
			}
		} else {
			for (int i = 1; i < min_pdr.length; i++) {
				min_pdr[i] = init_min_pdr;
			}
		}

		for (episode_no = 0; episode_no < no_episodes; episode_no++) {
			if (printEpisode) {
				System.out.println("Episode " + episode_no);
				System.out.println("-----------");
			}
			resetSimulator();
			initSimulator();
			ArrayList<RSU> templist = new ArrayList<RSU>();
			for (int i = 0; i < rsulist.size(); i++) {
				ArrayList<RSU> row = rsulist.get(i);
				for (int j = 0; j < row.size(); j++) {
					templist.add(row.get(j));
				}
			}
			ArrayList<RSU> Uprime;
			if (uniform) {
				Uprime = templist;
			} else {
				Uprime = RPR(templist, min_pdr[episode_no]);
			}
			if (trackRSU) {
				for (int i = 0; i < Uprime.size(); i++) {
					System.out.print(Uprime.get(i).id + ", ");
				}
				System.out.println("selected");
			}
			int maxsize = 0;
			int maxlen = 0;
			for (timestep_no = 0; timestep_no < no_timesteps; timestep_no++) {
				if (printTimestep) {
					System.out.println("t=" + timestep_no);
					System.out.println("-----------");
				}
				generateVehicles();
				changeVelocity();
				if (timestep_no % timeslot_len == 0) {
					toggleRSU(Uprime);
				}
				generatePackets();
				transmitPackets();
				receivePackets();
				if (printTimestep) {
					System.out.println("-----------");
				}
				for (Vehicle v : vehlist) {
					seg_dist[v.segment_x]++;
				}
				for (Vehicle v : vehlist) {
					maxsize = Math.max(maxsize, v.pktqueue.size());
				}
				maxlen = Math.max(maxlen, vehlist.size());
			}
			ArrayList<Integer> remainder = new ArrayList<Integer>();
			for (Vehicle v : vehlist) {
				for (Packet p : v.pktqueue) {
					if (!remainder.contains(p.id)) {
						remainder.add(p.id);
					}
				}
			}
			if (finalStats) {
				System.out.println("Packets received:" + recvd + ", generated:"
						+ generated + ", dropped:" + dropped + ", remaining:"
						+ remainder.size() + ". PDR = " + (float) recvd
						/ generated + ", V2I pkt delay = " + (float) recv_time
						/ recvd + ", OPEX = " + opex + ", Total Energy = "
						+ total_energy);
			}
			avgpdr += (double) recvd / generated;
			avgrxtime += (double) recv_time / recvd;
			avgopex += opex;
			avgenergy += total_energy;
			if (finalStats) {
				System.out.println("Longest queue: " + maxsize
						+ ", Max No. of Cars: " + maxlen + ", No. of RSUs: "
						+ Uprime.size());
			}
		}

		// Avg stats collected
		for (int i = 0; i < veh_dist.length; i++) {
			veh_dist[i] /= no_episodes;
		}
		// Computing avg segment density per km (hence the segment/1000)
		for (int i = 0; i < seg_dist.length; i++) {
			seg_dist[i] /= (no_episodes * no_timesteps * segment_len_x / 1000);
		}
		avgpdr /= no_episodes;
		avgrxtime /= no_episodes;
		avgopex /= no_episodes;
		avgenergy /= no_episodes;

		if (genGraphs) {
		//	plotSegdist();
		//	plotGraphs();
		}
		if (printData) {
			System.out.println("vehdist " + episode_no);
			for (int i = 0; i < veh_dist.length; i++) {
				System.out.println(i + "," + veh_dist[i]);
			}
			System.out.println("segdist " + episode_no);
			for (int i = 0; i < seg_dist.length; i++) {
				System.out.println(i + "," + seg_dist[i]);
			}
		}
		if (finalStats) {
			System.out.println("Simulation completed. Avg. PDR = " + avgpdr
					+ ", Avg. V2I pkt delay = " + avgrxtime + ", Avg. OPEX = "
					+ avgopex + ", Avg. Total Energy = 0" + avgenergy);
		}
		if (printAvgPDR) {
			System.out.println(avgpdr);
		}
	}

	/**
	 * Function that handles the reception (of currently broadcast packets in a
	 * segment) by both RSUs and vehicles.
	 */
	private static void receivePackets() {
		if (timestep_no % timeslot_len == 0) {
			for (int i = 0; i < (road_len_x / segment_len_x); i++) {
				for (int j = 0; j < (road_len_y / segment_len_y); j++) {
					num_delivered[i][j] = 0;
				}
			}
		}
		for (Vehicle v : vehlist) {
			// get transmitting vehicle's ID
			int tx_id = rsulist.get(v.segment_x).get(v.segment_y).idle;
			if (trackRSU) {
				System.out.println(tx_id + " holds segment " + v.segment_x
						+ "," + v.segment_y);
			}
			// if there's a vehicle tx_id that v can receive from
			if (tx_id >= 0 && tx_id != v.id) {
				Vehicle tx_vehicle = getByID(vehlist, tx_id);
				if (tx_id == v.tx_id
						&& tx_vehicle.pktqueue.peek().id == v.pkt_id) {
					// same vehicle as before, same pkt
					v.recv_tx--;
					if (v.recv_tx == 0) {
						// Pkt has been received completely. Add to queue and
						// reset tx vars
						Packet newcopy = tx_vehicle.pktqueue.peek().clone();
						newcopy.ttl = max_ttl;
						v.pktqueue.add(newcopy);
						if (trackPkt) {
							System.out.println("pkt "
									+ tx_vehicle.pktqueue.peek().id
									+ " of vehicle " + tx_vehicle.id
									+ " copied to " + v.id);
						}
						v.tx_id = -1;
						v.pkt_id = -1;
					}
				} else { // previous vehicle has gone away or different pkt from
							// same car
					v.recv_tx = tx_vehicle.pktqueue.peek().tx_time;
					v.tx_id = tx_id;
					v.pkt_id = tx_vehicle.pktqueue.peek().id;
					if (trackPkt) {
						System.out.println("new pkt rx:" + v.pkt_id
								+ " of vehicle " + v.tx_id + " by vehicle"
								+ v.id);
					}
				}
			} else {
				// vehicle v isn't receiving anything since there's no tx_id
				// around to rx from
				v.tx_id = -1;
				v.pkt_id = -1;
			}
		}
		for (int i = 0; i < rsulist.size(); i++) {
			ArrayList<RSU> row = rsulist.get(i);
			for (int j = 0; j < row.size(); j++) {
				RSU r = row.get(j);
				// Bother about rx only if rsu is on and
				// someone is transmitting
				if (r.on && r.idle >= 0) {
					// System.out.println("SINR " + calculateSINR(r));
					// bother about tx only if SINR is above threshold
					if (calculateSINR(r) > SINR_THRESH) {
						Vehicle tx_vehicle = getByID(vehlist, r.idle);
						if (r.idle == r.tx_id
								&& tx_vehicle.pktqueue.peek().id == r.pkt_id) {
							// same vehicle as before, same pkt
							r.recv_tx--;
							if (r.recv_tx == 0) {
								// Pkt has been received completely. Remove pkt
								// from
								// all queues and reset tx vars
								int to_remove = tx_vehicle.pktqueue.peek().id;
								recv_time += (timestep_no - tx_vehicle.pktqueue
										.peek().create_time);
								recvd++;
								num_delivered[i][j]++;
								for (Vehicle v : vehlist) {
									// if vehicle v is txing this pkt currently
									// stop the tx
									if (v.tx
											&& v.pktqueue.peek().id == to_remove) {
										v.tx = false;
										RSU rx_rsu = rsulist.get(v.segment_x)
												.get(v.segment_y);
										rx_rsu.tx_id = -1;
										rx_rsu.pkt_id = -1;
										rx_rsu.idle = -1;
										// backoff randomly for next tx
										v.backoff = rand.nextInt(cw_max
												- cw_min)
												+ cw_min;
									}
									boolean removed = delPktID(v.pktqueue,
											to_remove);
									if (removed) {
										if (trackPkt) {
											System.out.println("pkt "
													+ to_remove
													+ " of vehicle " + v.id
													+ " deleted");
										}
									}
								}
								if (trackRSU) {
									System.out.println("RSU " + r.id
											+ " recv pkt " + to_remove);
								}
								if (trackVehicle) {
									System.out.println("Vehicle "
											+ tx_vehicle.id + " delivered pkt "
											+ to_remove + " to RSU " + r.id
											+ " and backoff="
											+ tx_vehicle.backoff);
								}
							}
						} else {
							// previous vehicle has gone away or different pkt
							// from
							// same car
							r.recv_tx = tx_vehicle.pktqueue.peek().tx_time;
							r.tx_id = r.idle;
							r.pkt_id = tx_vehicle.pktqueue.peek().id;
							if (trackRSU) {
								System.out.println("new pkt rx:" + r.pkt_id
										+ " of vehicle " + r.tx_id + " by RSU "
										+ r.id);
							}
						}
					}
				} else {
					// RSU r isn't receiving anything since there's no tx_id
					// around to rx from
					r.tx_id = -1;
					r.pkt_id = -1;
				}
			}
		}
	}

	/**
	 * Returns the object with the given id in the passed list of vehicles.
	 * 
	 * @param vehlist
	 *            the list of vehicles
	 * @param id
	 *            id of the vehicle to be found
	 * @return null or Vehicle object
	 */
	private static Vehicle getByID(ArrayList<Vehicle> vehlist, int id) {
		// Get a vehicle by its internal ID
		for (Vehicle v : vehlist) {
			if (v.id == id) {
				return v;
			}
		}
		return null;
	}

	/**
	 * Deletes a packet with the given id in the passed packet queue
	 * 
	 * @param queue
	 *            the packet queue of a vehicle
	 * @param id
	 *            the packet to be deleted
	 * @return Boolean indicating success/failure of the delete task
	 */
	private static boolean delPktID(Queue<Packet> queue, int id) {
		for (Iterator<Packet> iterator = queue.iterator(); iterator.hasNext();) {
			Packet p = iterator.next();
			if (p.id == id) {
				iterator.remove();
				return true;
			}
		}
		return false;
	}

	/**
	 * Turns RSUs on if there is at least one vehicle in the corresponding
	 * segment. Else turns them off. Also updates energy stats, battery status,
	 * whether an RSU is using solar power or grid power. The passed parameter
	 * Uprime is the set of 'installed' RSUs; any RSU not in this list is
	 * permanently turned off.
	 * 
	 * @param Uprime
	 *            The list of installed RSUs
	 */
	private static void toggleRSU(ArrayList<RSU> Uprime) {
		// Turn RSU on if there is at least one vehicle in the corresponding
		// segment.
		// Else off.
		int[][] rsucount = new int[rsulist.size()][rsulist.get(0).size()];
		for (Vehicle v : vehlist) {
			rsucount[v.segment_x][v.segment_y]++;
		}
		for (int i = 0; i < rsulist.size(); i++) {
			for (int j = 0; j < rsulist.get(i).size(); j++) {
				RSU r = rsulist.get(i).get(j);
				boolean contained = false;
				for (RSU x : Uprime) {
					if (r.id == x.id) {
						contained = true;
						break;
					}
				}
				if (contained) {
					// RSU in Uprime, compute values
					int delta_e_serve = num_delivered[i][j] * tx_delay * rx_pow
							+ (timeslot_len - num_delivered[i][j] * tx_delay)
							* res_pow;
					int flip = 0;
					if (rsucount[i][j] == 0) {
						if (r.on != false) {
							r.on = false;
							flip = 1;
							if (trackRSU) {
								System.out.println("RSU " + r.id
										+ " turned off");
							}
						}
					} else {
						if (r.on != true) {
							r.on = true;
							flip = 1;
							if (trackRSU) {
								System.out
										.println("RSU " + r.id + " turned on");
							}
						}
					}
					// Update battery
					int r_on = r.on ? 1 : 0;
					r.solar = (r.battery > battery_delta) ? true : false;
					int r_solar = r.solar ? 1 : 0;
					r.battery += battery_profile - r_solar
							* (r_on * delta_e_serve + flip * delta_e_switch);
					if (r.battery < 0) {
						r.battery = 0;
					} else if (r.battery > max_battery) {
						r.battery = max_battery;
					}
					opex += (delta_e_serve + flip * delta_e_switch)
							* ((1 - r_solar) * grid_cost + r_solar * solar_cost);
					total_energy += delta_e_serve;
				}
			}
		}
	}
	/**
	 * Plots the arrival distribution of vehicles.
	 */
	/*
	private static void plotGraphs() {
		// Plots vehicle arrival distribution vs time
		int width = 640;
		int height = 480;
		XYSeries vehdist = new XYSeries("Vehicle Dist");
		for (int j = 0; j < veh_dist.length; j++) {
			vehdist.add(j, veh_dist[j]);
		}
		XYDataset dataset = new XYSeriesCollection(vehdist);
		JFreeChart chart = ChartFactory.createXYLineChart(
				"Vehicle Arrival Distribution", "Seconds", "No. of Vehicles",
				dataset, PlotOrientation.VERTICAL, false, false, false);
		File lineChart = new File("vehdist.jpeg");
		try {
			ChartUtilities.saveChartAsJPEG(lineChart, chart, width + 200,
					height + 200);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
*/
	/**
	 * Plots the avg segment density per km
	 */
	/*
	private static void plotSegdist() {
		// Plot no. of vehicles vs segment no.
		int width = 640;
		int height = 480;
		XYSeries segdist = new XYSeries("Segment Dist");
		for (int j = 0; j < seg_dist.length; j++) {
			segdist.add(j, seg_dist[j]);
		}
		XYDataset dataset = new XYSeriesCollection(segdist);
		JFreeChart chart = ChartFactory.createXYLineChart(
				"Vehicle Segment Distribution", "Segment No.",
				"No. of Vehicles", dataset, PlotOrientation.VERTICAL, false,
				false, false);
		File lineChart = new File("segdist.jpeg");
		try {
			ChartUtilities.saveChartAsJPEG(lineChart, chart, width, height);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
*/
	/**
	 * Resets all variables after every episode
	 */
	private static void resetSimulator() {
		// clear all data
		for (Vehicle v : vehlist) {
			v.pktqueue.clear();
		}
		vehlist.clear();
		rsulist.clear();
		generated = dropped = recvd = recv_time = 0;
		for (int i = 0; i < (road_len_x / segment_len_x); i++) {
			for (int j = 0; j < (road_len_y / segment_len_y); j++) {
				num_delivered[i][j] = 0;
			}
		}
		opex = 0.0;
		total_energy = 0.0;
		Vehicle.resetCounter();
		Packet.resetCounter();
		RSU.resetCounter();
	}

	/**
	 * Generates a 2D grid of RSUs
	 */
	private static void initSimulator() {
		// initialize the simulator with RSUs in every segment in the grid
		for (int i = 0; i < (road_len_x / segment_len_x); i++) {
			rsulist.add(new ArrayList<RSU>());
			for (int j = 0; j < (road_len_y / segment_len_y); j++) {
				rsulist.get(i).add(new RSU(i, j));
			}
		}
	}

	/**
	 * Handles the transmission of packets from vehicles. Drops packets whose
	 * delay bound is reached. Checks if backoff counter is zero, if no tx in 3
	 * hop neighbourhood and if no tx at current node, and then whether to tx or
	 * backoff.
	 */
	private static void transmitPackets() {
		ArrayList<ArrayList<ArrayList<Vehicle>>> tx_candidate = new ArrayList<ArrayList<ArrayList<Vehicle>>>();
		for (int i = 0; i < (road_len_x / segment_len_x); i++) {
			tx_candidate.add(new ArrayList<ArrayList<Vehicle>>());
			for (int j = 0; j < (road_len_y / segment_len_y); j++) {
				tx_candidate.get(i).add(new ArrayList<Vehicle>());
			}
		}

		for (Vehicle v : vehlist) {
			// Drop packets whose delay bound is reached
			for (Iterator<Packet> iterator = v.pktqueue.iterator(); iterator
					.hasNext();) {
				Packet p = iterator.next();
				p.ttl--;
				if (trackPkt) {
					System.out.println("pkt " + p.id + " of vehicle " + v.id
							+ " ttl=" + p.ttl);
				}
				if (p.ttl == 0) {
					if (v.tx) {
						// if the pkt p is the currently transmitted queue head
						if (p == v.pktqueue.peek()) {
							// stop tx
							v.tx = false;
							rsulist.get(v.segment_x).get(v.segment_y).idle = -1;
							// backoff randomly for next tx
							v.backoff = rand.nextInt(cw_max - cw_min) + cw_min;
						}
					}
					if (trackPkt) {
						System.out.println("pkt " + p.id + " of vehicle "
								+ v.id + " dropped");
					}
					dropped++;
					iterator.remove();
				}
			}

			if (trackVehicle) {
				System.out.println("Vehicle " + v.id + " backoff " + v.backoff);
			}

			// If vehicle isn't transmitting now and has a pkt to transmit
			// and no tx in 3 hop nbhd in 2D
			if (!v.tx && v.pktqueue.size() > 0 && checkNbhdIdle(rsulist, v)) {
				v.backoff--;
				if (v.backoff == 0) {
					if (trackVehicle) {
						System.out.println("Vehicle " + v.id
								+ " ready to tx pkt " + v.pktqueue.peek().id);
					}
					// add v as a candidate for its segment
					tx_candidate.get(v.segment_x).get(v.segment_y).add(v);
				}
			}
		}

		for (ArrayList<ArrayList<Vehicle>> row : tx_candidate) {
			for (ArrayList<Vehicle> seglist : row) {
				if (seglist.size() > 0) {
					// vehicle that gets to tx
					Vehicle veh_tx = seglist.get(rand.nextInt(seglist.size()));
					if (trackVehicle) {
						System.out.println("Vehicle " + veh_tx.id
								+ " selected to tx pkt "
								+ veh_tx.pktqueue.peek().id);
					}
					for (Vehicle v : seglist) {
						if (v == veh_tx) {
							rsulist.get(v.segment_x).get(v.segment_y).idle = v.id;
							v.tx = true;
						} else {
							v.backoff = rand.nextInt(cw_max - cw_min) + cw_min;
							if (trackVehicle) {
								System.out.println("Vehicle " + v.id
										+ " backoff for " + v.backoff);
							}
						}
					}
				}
			}
		}
	}

	/**
	 * Checks of the num_hops hop neighbourhood of a vehicle v is tx-free, by
	 * checking if there is any tx at the neighbouring RSUs.
	 * 
	 * @param rsulist
	 *            the list of RSUs
	 * @param v
	 *            The vehicle whose neighbourhood is to be checked
	 * @return A boolean indicating if the neighbourhood of vehicle v is idle or
	 *         not.
	 */
	private static boolean checkNbhdIdle(ArrayList<ArrayList<RSU>> rsulist,
			Vehicle v) {
		for (int i = Math.max(0, v.segment_x - num_hops); i <= Math.min(
				(road_len_x / segment_len_x) - 1, v.segment_x + num_hops); i++) {
			if (rsulist.get(i).get(v.segment_y).idle >= 0) {
				return false;
			}
		}
		for (int i = Math.max(0, v.segment_y - num_hops); i <= Math.min(
				(road_len_y / segment_len_y) - 1, v.segment_y + num_hops); i++) {
			if (rsulist.get(v.segment_x).get(i).idle >= 0) {
				return false;
			}
		}
		return true;
	}

	/**
	 * Generates a random double sampled from an exponential distribution with
	 * parameter lambda. Performs inverse transform sampling.
	 * 
	 * @param lambda
	 * @return A random number sampled from the exponential distribution
	 */
	public static double generateExp(double lambda) {
		// Generate random var sampled from exp dist
		// Computed using inverse sampling method
		if (lambda <= 0) {
			throw new IllegalArgumentException(
					"Lambda must be greater than zero!");
		}
		return Math.log(1 - rand.nextFloat()) / (-lambda);
	}

	/**
	 * Generates new vehicles, updates positions of old vehicles, checks for
	 * boundary conditions, decides if to loop them (fixed_cars) or to let them
	 * exit the grid.
	 */
	private static void generateVehicles() {
		// Compute new posn for all vehicles and remove those that have left the
		// grid
		for (Iterator<Vehicle> iterator = vehlist.iterator(); iterator
				.hasNext();) {
			Vehicle v = iterator.next();
			v.pos_x += v.x_dxn * v.velocity;
			v.pos_y += v.y_dxn * v.velocity;
			if (trackVehicle) {
				System.out.println("Vehicle " + v.id + " new pos (" + v.pos_x
						+ "," + v.pos_y + ")");
			}
			int new_seg_x = (int) (v.pos_x / segment_len_x);
			int new_seg_y = (int) (v.pos_y / segment_len_y);
			// Vehicle outside boundaries
			if (new_seg_x >= (road_len_x / segment_len_x) || new_seg_x < 0
					|| new_seg_y >= (road_len_y / segment_len_y)
					|| new_seg_y < 0) {
				if (fixed_cars) {
					// Loop the vehicle
					// Put it at the border
					if (new_seg_x >= (road_len_x / segment_len_x)) {
						new_seg_x = (road_len_x / segment_len_x) - 1;
					} else if (new_seg_x < 0) {
						new_seg_x = 0;
					}
					if (new_seg_y >= (road_len_y / segment_len_y)) {
						new_seg_y = (road_len_y / segment_len_y) - 1;
					} else if (new_seg_y < 0) {
						new_seg_y = 0;
					}
					// Change its direction
					v.x_dxn *= -1;
					v.y_dxn *= -1;
					if (v.tx) {
						rsulist.get(v.segment_x).get(v.segment_y).idle = -1;
						rsulist.get(new_seg_x).get(new_seg_y).idle = v.id;
					}
					if (trackVehicle) {
						System.out.println("Vehicle " + v.id
								+ " tx seg change from (" + v.segment_x + ","
								+ v.segment_y + ") to (" + new_seg_x + ","
								+ new_seg_y + ")");
					}
					v.segment_x = new_seg_x;
					v.segment_y = new_seg_y;
					if (trackVehicle) {
						System.out.println("Vehicle " + v.id + " new seg ("
								+ v.segment_x + "," + v.segment_y + ")");
					}
				} else {
					if (v.tx) {
						rsulist.get(v.segment_x).get(v.segment_y).idle = -1;
					}
					if (trackVehicle) {
						System.out.println("Vehicle " + v.id + " deleted");
					}
					iterator.remove();
				}
			} else {
				if ((v.segment_x != new_seg_x || v.segment_y != new_seg_y)
						&& v.tx) {
					if (v.tx) {
						rsulist.get(v.segment_x).get(v.segment_y).idle = -1;
						rsulist.get(new_seg_x).get(new_seg_y).idle = v.id;
					}
					if (trackVehicle) {
						System.out.println("Vehicle " + v.id
								+ " tx seg change from (" + v.segment_x + ","
								+ v.segment_y + ") to (" + new_seg_x + ","
								+ new_seg_y + ")");
					}
				}
				v.segment_x = new_seg_x;
				v.segment_y = new_seg_y;
				if (trackVehicle) {
					System.out.println("Vehicle " + v.id + " new seg ("
							+ v.segment_x + "," + v.segment_y + ")");
				}
			}
		}

		if (fixed_cars) {
			if (timestep_no == 0) {
				// Generate fixed no. of vehicles along X and Y axes, in both
				// dxns
				int no_generated = fixed_cars_no;
				veh_dist[timestep_no] += no_generated;

				if (trackVehicle) {
					System.out.println(vehlist.size() + " vehicles present.");
					System.out.println(no_generated
							+ " vehicles gen from L to R:");
				}
				for (int i = 0; i < no_generated; i++) {
					Vehicle to_add = new Vehicle(
							generateExp(lambda_velocity),
							+1,
							0,
							0,
							segment_len_y
									* rand.nextInt((int) (road_len_y / segment_len_y)));
					vehlist.add(to_add);
					if (trackVehicle) {
						System.out.println("Vehicle " + to_add.id);
					}
				}

				no_generated = fixed_cars_no;
				if (trackVehicle) {
					System.out.println(no_generated
							+ " vehicles gen from R to L:");
				}
				for (int i = 0; i < no_generated; i++) {
					Vehicle to_add = new Vehicle(
							generateExp(lambda_velocity),
							-1,
							0,
							road_len_x - 1,
							segment_len_y
									* rand.nextInt((int) (road_len_y / segment_len_y)));
					vehlist.add(to_add);
					if (trackVehicle) {
						System.out.println("Vehicle " + to_add.id);
					}
				}
				if (vert_traffic) {
					no_generated = fixed_cars_no;
					if (trackVehicle) {
						System.out.println(no_generated
								+ " vehicles gen from T to B:");
					}
					for (int i = 0; i < no_generated; i++) {
						Vehicle to_add = new Vehicle(
								generateExp(lambda_velocity),
								0,
								+1,
								segment_len_x
										* rand.nextInt((int) (road_len_x / segment_len_x)),
								0);
						vehlist.add(to_add);
						if (trackVehicle) {
							System.out.println("Vehicle " + to_add.id);
						}
					}

					no_generated = fixed_cars_no;
					if (trackVehicle) {
						System.out.println(no_generated
								+ " vehicles gen from B to T:");
					}
					for (int i = 0; i < no_generated; i++) {
						Vehicle to_add = new Vehicle(
								generateExp(lambda_velocity),
								0,
								-1,
								segment_len_x
										* rand.nextInt((int) (road_len_x / segment_len_x)),
								road_len_y - 1);
						vehlist.add(to_add);
						if (trackVehicle) {
							System.out.println("Vehicle " + to_add.id);
						}
					}
				}
			}
		} else {
			// Generate random no. of vehicles along X and Y axes, in both dxns
			int no_generated = (int) generateExp(lambda_vehicle);
			veh_dist[timestep_no] += no_generated;

			if (trackVehicle) {
				System.out.println(vehlist.size() + " vehicles present.");
				System.out.println(no_generated + " vehicles gen from L to R:");
			}
			for (int i = 0; i < no_generated; i++) {
				Vehicle to_add = new Vehicle(
						generateExp(lambda_velocity),
						+1,
						0,
						0,
						segment_len_y
								* rand.nextInt((int) (road_len_y / segment_len_y)));
				vehlist.add(to_add);
				if (trackVehicle) {
					System.out.println("Vehicle " + to_add.id);
				}
			}

			no_generated = (int) generateExp(lambda_vehicle);
			if (trackVehicle) {
				System.out.println(no_generated + " vehicles gen from R to L:");
			}
			for (int i = 0; i < no_generated; i++) {
				Vehicle to_add = new Vehicle(
						generateExp(lambda_velocity),
						-1,
						0,
						road_len_x - 1,
						segment_len_y
								* rand.nextInt((int) (road_len_y / segment_len_y)));
				vehlist.add(to_add);
				if (trackVehicle) {
					System.out.println("Vehicle " + to_add.id);
				}
			}
			if (vert_traffic) {
				no_generated = (int) generateExp(lambda_vehicle);
				if (trackVehicle) {
					System.out.println(no_generated
							+ " vehicles gen from T to B:");
				}
				for (int i = 0; i < no_generated; i++) {
					Vehicle to_add = new Vehicle(
							generateExp(lambda_velocity),
							0,
							+1,
							segment_len_x
									* rand.nextInt((int) (road_len_x / segment_len_x)),
							0);
					vehlist.add(to_add);
					if (trackVehicle) {
						System.out.println("Vehicle " + to_add.id);
					}
				}

				no_generated = (int) generateExp(lambda_vehicle);
				if (trackVehicle) {
					System.out.println(no_generated
							+ " vehicles gen from B to T:");
				}
				for (int i = 0; i < no_generated; i++) {
					Vehicle to_add = new Vehicle(
							generateExp(lambda_velocity),
							0,
							-1,
							segment_len_x
									* rand.nextInt((int) (road_len_x / segment_len_x)),
							road_len_y - 1);
					vehlist.add(to_add);
					if (trackVehicle) {
						System.out.println("Vehicle " + to_add.id);
					}
				}
			}
		}
	}

	/**
	 * Generates packets for all vehicles in vehlist by sampling an exponential
	 * distribution.
	 */
	private static void generatePackets() {
		for (Vehicle v : vehlist) {
			int no_generated = (int) generateExp(lambda_pkt);
			for (int i = 0; i < no_generated; i++) {
				Packet newpkt = new Packet(v.id, pkt_size);
				v.pktqueue.add(newpkt);
				generated++;
				if (trackPkt) {
					System.out.println("pkt " + newpkt.id
							+ " added to vehicle " + v.id);
				}
			}
		}
	}

	/**
	 * Changes the velocity of vehicles in vehlist by making use of each
	 * vehicle's timer, and samples a new velocity from an exponential
	 * distribution once the countdown reaches zero.
	 */
	private static void changeVelocity() {
		for (Vehicle v : vehlist) {
			if (v.checkTimer()) {
				double new_velocity = generateExp(lambda_velocity);
				if (trackVehicle) {
					System.out.println("Vehicle " + v.id
							+ " velocity changed from " + v.velocity + " to "
							+ new_velocity);
				}
				v.velocity = new_velocity;
			}
		}
	}

	/**
	 * A method called only by the RPR algorithm, meant for toggling RSUs as in
	 * toggleRSU(). rsulist is the list of RSUs that are available,
	 * num_delivered keeps track of the number of packets delivered in a time
	 * slot, u is the new RSU being considered (if to add to Uprime, the list of
	 * RSUs that are to be installed). Functioning is same as toggleRSU()
	 * mostly.
	 * 
	 * @param rsulist
	 *            the list of RSUs generated for the RPR simulation
	 * @param num_delivered
	 *            number of pkts delivered in the current timeslot
	 * @param u
	 *            the RSU under consideration
	 * @param Uprime
	 *            the RSUs already chosen
	 * @return Returns fraction of grid energy consumed and the energy consumed
	 *         by RSU u as an array
	 */
	private static double[] _toggleRSU(ArrayList<ArrayList<RSU>> rsulist,
			int[][] num_delivered, RSU u, ArrayList<RSU> Uprime) {
		double num = 0.0;
		double den = 0.0;
		double energy_u = 0.0;

		for (int i = 0; i < rsulist.size(); i++) {
			for (int j = 0; j < rsulist.get(i).size(); j++) {
				int delta_e_serve = num_delivered[i][j] * tx_delay * rx_pow
						+ (timeslot_len - num_delivered[i][j] * tx_delay)
						* res_pow;
				RSU r = rsulist.get(i).get(j);
				int flip = 0;
				boolean contained = false;
				for (RSU x : Uprime) {
					if (r.id == x.id) {
						contained = true;
						break;
					}
				}
				if (r.id == u.id || contained) {
					if (r.on != true) {
						r.on = true;
						flip = 1;
					}
				} else {
					if (r.on != false) {
						r.on = false;
						flip = 1;
					}
				}
				// Update battery
				int r_on = r.on ? 1 : 0;
				r.solar = (r.battery > battery_delta) ? true : false;
				int r_solar = r.solar ? 1 : 0;
				r.battery += battery_profile - r_solar
						* (r_on * delta_e_serve + flip * delta_e_switch);
				if (r.battery < 0) {
					r.battery = 0;
				} else if (r.battery > max_battery) {
					r.battery = max_battery;
				}
				num += (delta_e_serve + flip * delta_e_switch) * (1 - r_solar);
				den += delta_e_serve;
				if (r == u) {
					energy_u = delta_e_serve;
				}
			}
		}
		double[] retval = new double[2];
		retval[0] = num / den;
		retval[1] = energy_u;
		return retval;
	}

	/**
	 * Method called by the RPR algorithm to generate vehicles and change speeds
	 * and generate packets for vehicles. Vehicles are generated along a 1D road
	 * only. rsulist is the entire list of RSUs, vehlist is the list of vehicles
	 * and u is the RSU under consideration currently in the RPR algorithm.
	 * Functioning is same as generateVehicles, generatePackets() and
	 * changeVelocity() mostly.
	 * 
	 * @param rsulist
	 *            the list of RSUs generated for the RPR simulation
	 * @param vehlist
	 *            the list of vehicles generated for the RPR simulation
	 * @param u
	 *            the RSU under consideration
	 * @return Number of packets generated
	 */
	private static int _generateVehicles(ArrayList<ArrayList<RSU>> rsulist,
			ArrayList<Vehicle> vehlist, RSU u) {
		int generated = 0;
		for (Iterator<Vehicle> iterator = vehlist.iterator(); iterator
				.hasNext();) {
			Vehicle v = iterator.next();
			v.pos_x += v.x_dxn * v.velocity;
			v.pos_y += v.y_dxn * v.velocity;
			int new_seg_x = (int) (v.pos_x / segment_len_x);
			int new_seg_y = (int) (v.pos_y / segment_len_y);
			if (new_seg_x >= (road_len_x / segment_len_x) || new_seg_x < 0
					|| new_seg_y >= (road_len_y / segment_len_y)
					|| new_seg_y < 0) {
				if (v.tx) {
					rsulist.get(v.segment_x).get(v.segment_y).idle = -1;
				}
				iterator.remove();
			} else {
				if ((v.segment_x != new_seg_x || v.segment_y != new_seg_y)
						&& v.tx) {
					if (v.tx) {
						rsulist.get(v.segment_x).get(v.segment_y).idle = -1;
						rsulist.get(new_seg_x).get(new_seg_y).idle = v.id;
					}
				}
				v.segment_x = new_seg_x;
				v.segment_y = new_seg_y;
			}
		}
		int no_generated = (int) generateExp(lambda_vehicle);
		for (int i = 0; i < no_generated; i++) {
			vehlist.add(new Vehicle(generateExp(lambda_velocity), +1, 0, 0,
					segment_len_y
							* rand.nextInt((int) (road_len_y / segment_len_y))));
		}
		// System.out.println(no_generated+"vehs from L to R");
		no_generated = (int) generateExp(lambda_vehicle);
		for (int i = 0; i < no_generated; i++) {
			vehlist.add(new Vehicle(generateExp(lambda_velocity), -1, 0,
					road_len_x - 1, segment_len_y
							* rand.nextInt((int) (road_len_y / segment_len_y))));
		}
		// System.out.println(no_generated+"vehs from R to L");
		for (Vehicle v : vehlist) {
			if (v.checkTimer()) {
				v.velocity = generateExp(lambda_velocity);
			}
			no_generated = (int) generateExp(lambda_pkt);
			for (int i = 0; i < no_generated; i++) {
				v.pktqueue.add(new Packet(v.id, pkt_size));
				generated++;
			}
		}
		return generated;
	}

	/**
	 * Method called by RPR algorithm. Helps transmit packets from vehicles.
	 * Parameters are the entire list of RSUs, rsulist and the list of vehicles,
	 * vehlist. Functioning is same as transmitPackets() mostly.
	 * 
	 * @param rsulist
	 *            the list of RSUs generated for the RPR simulation
	 * @param vehlist
	 *            the list of vehilces generated for the RPR simulation
	 */
	private static void _transmitPackets(ArrayList<ArrayList<RSU>> rsulist,
			ArrayList<Vehicle> vehlist) {
		ArrayList<ArrayList<ArrayList<Vehicle>>> tx_candidate = new ArrayList<ArrayList<ArrayList<Vehicle>>>();
		for (int i = 0; i < (road_len_x / segment_len_x); i++) {
			tx_candidate.add(new ArrayList<ArrayList<Vehicle>>());
			for (int j = 0; j < (road_len_y / segment_len_y); j++) {
				tx_candidate.get(i).add(new ArrayList<Vehicle>());
			}
		}
		for (Vehicle v : vehlist) {
			for (Iterator<Packet> iterator = v.pktqueue.iterator(); iterator
					.hasNext();) {
				Packet p = iterator.next();
				p.ttl--;
				if (p.ttl == 0) {
					if (v.tx) {
						if (p == v.pktqueue.peek()) {
							v.tx = false;
							rsulist.get(v.segment_x).get(v.segment_y).idle = -1;
							v.backoff = rand.nextInt(cw_max - cw_min) + cw_min;
						}
					}
					iterator.remove();
				}
			}
			if (!v.tx && v.pktqueue.size() > 0 && checkNbhdIdle(rsulist, v)) {
				v.backoff--;
				if (v.backoff == 0) {
					tx_candidate.get(v.segment_x).get(v.segment_y).add(v);
				}
			}
		}

		for (ArrayList<ArrayList<Vehicle>> row : tx_candidate) {
			for (ArrayList<Vehicle> seglist : row) {
				if (seglist.size() > 0) {
					// vehicle that gets to tx
					Vehicle veh_tx = seglist.get(rand.nextInt(seglist.size()));
					for (Vehicle v : seglist) {
						if (v == veh_tx) {
							rsulist.get(v.segment_x).get(v.segment_y).idle = v.id;
							v.tx = true;
						} else {
							v.backoff = rand.nextInt(cw_max - cw_min) + cw_min;
						}
					}
				}
			}
		}
	}

	/**
	 * Helps vehicles deliver packets to other vehicles and RSUs. Used by the
	 * RPR algorithm. Parameters are the RSU list rsulist, and the list of
	 * vehicles, vehlist, the number of pkts delivered in the current time slot
	 * and the RSU under consideration, u. Functioning is same as
	 * receivePackets() mostly.
	 * 
	 * @param rsulist
	 *            the list of RSUs generated for the RPR simulation
	 * @param vehlist
	 *            the list of vehilces generated for the RPR simulation
	 * @param num_delivered
	 *            number of packets delivered in the current time slot
	 * @param u
	 *            the RSU under consideration
	 * @return The number of pkts received
	 */
	private static int _receivePackets(ArrayList<ArrayList<RSU>> rsulist,
			ArrayList<Vehicle> vehlist, int[][] num_delivered, RSU u) {
		int recvd = 0;
		if (timestep_no % timeslot_len == 0) {
			for (int i = 0; i < (road_len_x / segment_len_x); i++) {
				for (int j = 0; j < (road_len_y / segment_len_y); j++) {
					num_delivered[i][j] = 0;
				}
			}
		}
		for (Vehicle v : vehlist) {
			int tx_id = rsulist.get(v.segment_x).get(v.segment_y).idle;
			if (tx_id >= 0 && tx_id != v.id) {
				Vehicle tx_vehicle = getByID(vehlist, tx_id);
				if (tx_id == v.tx_id
						&& tx_vehicle.pktqueue.peek().id == v.pkt_id) {
					v.recv_tx--;
					if (v.recv_tx == 0) {
						Packet newcopy = tx_vehicle.pktqueue.peek().clone();
						newcopy.ttl = max_ttl;
						v.pktqueue.add(newcopy);
						v.tx_id = -1;
						v.pkt_id = -1;
					}
				} else {
					v.recv_tx = tx_vehicle.pktqueue.peek().tx_time;
					v.tx_id = tx_id;
					v.pkt_id = tx_vehicle.pktqueue.peek().id;
				}
			} else {
				v.tx_id = -1;
				v.pkt_id = -1;
			}
		}
		for (int i = 0; i < rsulist.size(); i++) {
			ArrayList<RSU> row = rsulist.get(i);
			for (int j = 0; j < row.size(); j++) {
				RSU r = row.get(j);
				if (r.on && r.idle >= 0) {
					Vehicle tx_vehicle = getByID(vehlist, r.idle);
					if (r.idle == r.tx_id
							&& tx_vehicle.pktqueue.peek().id == r.pkt_id) {
						r.recv_tx--;
						if (r.recv_tx == 0) {
							int to_remove = tx_vehicle.pktqueue.peek().id;
							recv_time += (timestep_no - tx_vehicle.pktqueue
									.peek().create_time);
							recvd++;
							num_delivered[i][j]++;
							for (Vehicle v : vehlist) {
								if (v.tx && v.pktqueue.peek().id == to_remove) {
									v.tx = false;
									RSU rx_rsu = rsulist.get(v.segment_x).get(
											v.segment_y);
									rx_rsu.tx_id = -1;
									rx_rsu.pkt_id = -1;
									rx_rsu.idle = -1;
									v.backoff = rand.nextInt(cw_max - cw_min)
											+ cw_min;
								}
								delPktID(v.pktqueue, to_remove);
							}
						}
					} else {
						r.recv_tx = tx_vehicle.pktqueue.peek().tx_time;
						r.tx_id = r.idle;
						r.pkt_id = tx_vehicle.pktqueue.peek().id;
					}
				} else {
					r.tx_id = -1;
					r.pkt_id = -1;
				}
			}
		}
		return recvd;
	}

	/**
	 * Takes the RSU under consideration by the RPR algorithm, u, and the RSUs
	 * to be installed, Uprime, and computes PDR, Energy and Grid fraction.
	 * 
	 * @param u
	 *            the RSU being considered
	 * @param Uprime
	 *            the RSUs already chosen
	 * @return PDR, Energy and Grid Fraction values for RSU u.
	 */
	public static double[] getStats(RSU u, ArrayList<RSU> Uprime) {
		// Generate local copies of global variables for RPR simulation
		// We don't want to mess up our global values
		int no_episodes = rpr_no_episodes;
		int no_timesteps = rpr_no_timesteps;
		double[] avgvals = new double[4];
		int[][] num_delivered = new int[road_len_x / segment_len_x][road_len_y
				/ segment_len_y];
		int generated = 0;
		int recvd = 0;
		double[] temparray = new double[2];
		RSU.resetCounter();
		ArrayList<ArrayList<RSU>> rsulist = new ArrayList<ArrayList<RSU>>();
		ArrayList<Vehicle> vehlist = new ArrayList<Vehicle>();

		// Generate a new RSU grid
		for (int i = 0; i < (road_len_x / segment_len_x); i++) {
			rsulist.add(new ArrayList<RSU>());
			for (int j = 0; j < (road_len_y / segment_len_y); j++) {
				rsulist.get(i).add(new RSU(i, j));
			}
		}

		// Obtain stats and average them
		for (int episode_no = 0; episode_no < no_episodes; episode_no++) {
			temparray = _toggleRSU(rsulist, num_delivered, u, Uprime);
			for (int timestep_no = 0; timestep_no < no_timesteps; timestep_no++) {
				generated += _generateVehicles(rsulist, vehlist, u);
				_transmitPackets(rsulist, vehlist);
				recvd += _receivePackets(rsulist, vehlist, num_delivered, u);
			}
			avgvals[0] += (double) recvd / generated; // pdr
			avgvals[1] += temparray[1]; // energy
			avgvals[2] += temparray[0]; // gridfraction
		}
		avgvals[0] /= no_episodes;
		avgvals[1] /= no_episodes;
		avgvals[2] /= no_episodes;
		int i = 0;
		int j = 0;
		boolean flag = false;
		for (i = 0; i < Simulator.rsulist.size(); i++) {
			ArrayList<RSU> row = Simulator.rsulist.get(i);
			for (j = 0; j < row.size(); j++) {
				if (row.get(j) == u) {
					flag = true;
					break;
				}
			}
			if (flag == true) {
				break;
			}
		}
		avgvals[3] = Simulator.rsulist.get(i).get(j).on ? 1 : 0;// placed
		return avgvals;
	}

	/**
	 * The modified RPR algorithm, implemented according to Vageesh's thesis.
	 * Takes the set of RSUs U, and the min_pdr required.
	 * 
	 * @param U
	 *            set of RSUs
	 * @param min_pdr
	 *            minimum PDR required
	 * @return The final list of RSUs that need to be installed.
	 */
	private static ArrayList<RSU> RPR(ArrayList<RSU> U, double min_pdr) {
		ArrayList<RSU> Uprime = new ArrayList<RSU>();
		boolean all_coverage_in_bound = false;
		while (!all_coverage_in_bound && U.size() != 0) {
			ArrayList<RSU> S = new ArrayList<RSU>();
			ArrayList<double[]> statlist = new ArrayList<double[]>();
			// get stats for all RSUs
			for (int i = 0; i < U.size(); i++) {
				RSU u = U.get(i);
				statlist.add(getStats(u, Uprime));
				// if min pdr isn't satisfied, add to set S
				if (statlist.get(i)[0] < min_pdr) {
					S.add(u);
				}
			}
			if (S.size() == 0) {
				all_coverage_in_bound = true;
				continue;
			}
			ArrayList<double[]> M = new ArrayList<double[]>();
			int currcount = 0;
			// Generate M, the corresponding feature set for S
			for (int i = 0; i < U.size() && currcount < S.size(); i++) {
				if (S.get(currcount).id == U.get(i).id) {
					currcount++;
					double[] stats = statlist.get(i);
					double[] temp = new double[3];
					temp[0] = stats[1]; // energy
					temp[1] = stats[2]; // gridFraction
					temp[2] = stats[3]; // placed
					M.add(temp);
				}
			}
			// obtain best RSU
			RSU s = Rainbow_product_ranking(S, M);
			// add best RSU to Uprime, remove from U
			Uprime.add(s);
			U.remove(s);
		}
		return Uprime;
	}

	/**
	 * A single interation of the RPR algorithm that the original paper talks
	 * about. Called by Vageesh's algorithm. Takes a list of RSUs S and their
	 * features M, to build a skyline and suggest a single RSU to pick based on
	 * the best features.
	 * 
	 * @param S
	 *            a list of RSUs
	 * @param M
	 *            the features of the RSUs
	 * @return The best candidate discovered by the RPR algorithm
	 */
	private static RSU Rainbow_product_ranking(ArrayList<RSU> S,
			ArrayList<double[]> M) {
		ArrayList<RSU> s = new ArrayList<RSU>();
		// Generate the best skyline set s
		for (int i = 0; i < S.size(); i++) {
			RSU x = S.get(i);
			double[] x_attr = M.get(i);
			boolean flag = true;
			for (int j = 0; j < S.size(); j++) {
				double[] y_attr = M.get(j);
				if (x_attr[0] > y_attr[0] && x_attr[1] > y_attr[1]
						&& x_attr[2] > y_attr[2]) {
					flag = false;
				}
			}
			if (flag == true) {
				s.add(x);
			}
		}
		// Final sorting based on feature 0 here. Choose best point from s.
		int best_candidate = 0;
		double[] best_attr = M.get(best_candidate);
		for (int i = 1; i < s.size(); i++) {
			double[] curr_attr = M.get(i);
			if (best_attr[0] > curr_attr[0]) {
				best_candidate = i;
				best_attr = curr_attr;
			}
		}
		return s.get(best_candidate);
	}

	/**
	 * Calculates the interference an RSU receives from every vehicle it isn't
	 * attmepting to receive from, in a num_hops neighborhood.
	 * 
	 * @param r
	 *            The receiving RSU
	 * @return The interference, in dBm
	 */
	private static double calculateInterference(RSU r) {

		double interference = 0, distance, signal, maxInterferer = -9999;

		// Iterate through all vehicles
		for (Vehicle v : vehlist) {
			// Consider only those in a num_hops hop neighborhood
			if (v.segment_x >= Math.max(0, r.segment_x - num_hops)
					&& v.segment_x <= Math.min(
							(road_len_x / segment_len_x) - 1, r.segment_x
									+ num_hops)
					&& v.segment_y >= Math.max(0, r.segment_y - num_hops)
					&& v.segment_y <= Math.min(
							(road_len_y / segment_len_y) - 1, r.segment_y
									+ num_hops)) {
				// Vehicles that aren't the target are interference
				if (r.idle != v.id) {
					distance = calcDistance(v.pos_x, v.pos_y,
							(r.segment_x * segment_len_x)
									+ (segment_len_x / 2.0),
							(r.segment_y * segment_len_y)
									+ (segment_len_y / 2.0));
					double PLoss = 110 + 37 * Math.log10(distance / 1000);
					// if(distance>10) PLoss += EXT_WALL_LOSS_DB;
					signal = TxPower - PLoss;
					if (signal > maxInterferer) {
						maxInterferer = signal;
					}
				}
				// Interference=max(all interference)
				interference = maxInterferer;
			}
		}
		return interference;
	}

	/**
	 * Computes the Euclidean distance between two 2D points
	 * 
	 * @param x1
	 *            x-coordinate of the first point.
	 * @param y1
	 *            y-coordinate of the first point.
	 * @param x2
	 *            x-coordinate of the second point.
	 * @param y2
	 *            y-coordinate of the second point.
	 * @return The euclidean distance between the 2 points.
	 */
	private static double calcDistance(double x1, double y1, double x2,
			double y2) {
		return Math.sqrt(((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
	}

	/**
	 * Computes the SINR faced by an RSU
	 * 
	 * @param r
	 *            The RSU under consideration
	 * @return The SINR value
	 */
	private static double calculateSINR(RSU r) {
		double interference = calculateInterference(r);
		double rcv_signal = recievedSignal(r);

		if (interference >= NOISE_DBM) {
			interference = 0;
		}
		double SINR = 2 * rcv_signal - interference - NOISE_DBM;
		if (SINR > 40) {
			SINR = 40;
		}
		if (SINR < -40) {
			SINR = -40;
		}
		return Math.pow(10, SINR / 10);
	}

	/**
	 * Computes the power of the received signal, by an RSU
	 * 
	 * @param r
	 *            The RSU under consideration
	 * @return The power of the received signal
	 */
	private static double recievedSignal(RSU r) {
		Vehicle tgt = getByID(vehlist, r.idle);
		double distance = calcDistance((r.segment_x * segment_len_x)
				+ (segment_len_x / 2.0), (r.segment_y * segment_len_y)
				+ (segment_len_y / 2.0), tgt.pos_x, tgt.pos_y);
		double PLoss = 110 + 37 * Math.log10(distance / 1000);
		double rxSignal = TxPower - PLoss;
		return rxSignal;
	}
}