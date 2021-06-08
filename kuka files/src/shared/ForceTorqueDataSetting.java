package shared;

import java.util.ArrayList;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
import com.kuka.task.ITaskLogger;

/**
 * This Thread calculate the current mass and update the mass of the linkage
 * with the calculated mass. If the measurement is not valid then it take the
 * last valid value and set it. If the calculate force is positive, example:
 * pushing upwards, then it set the last calculated mass.
 * 
 * @author Firas Amin
 * 
 */
public class ForceTorqueDataSetting {

	private LBR robot;
	private Tool linkage;
	private ITaskLogger logger;
	private boolean isFinished;
	private ObjectFrame tcp;
	private double forceInZ;
	private ArrayList<TorqueSensorData> register;
	private ArrayList<Frame> trajectory;
	protected ArrayList<Double> weights;
	private volatile double lastWeight = 0;
	private volatile boolean suspended;
	private ArrayList<Frame> frameList;

	public ForceTorqueDataSetting(ArrayList<Frame> _frameList, LBR _lbr, Tool _linkage, ITaskLogger _logger) {
		this.robot = _lbr;
		this.linkage = _linkage;
		this.logger = _logger;
		this.frameList = _frameList;
		setCurrentWeight();
		//ThreadUtil.milliSleep(100);
		register();
	}

	/**
	 * Calculate the current mass and update the mass of the linkage with the
	 * calculated mass. If the measurement is not valid then it take the last
	 * valid value and set it. If the calculate force is positive, example:
	 * pushing upwards, then it set the last calculated mass.
	 * 
	 */
	private void setCurrentWeight() {

		try {
			new Thread(new Runnable() {

				@Override
				public void run() {
					tcp = linkage.getFrame("/RecTCP");
					double myforce = calcForce();
					lastWeight = Math.abs(myforce / 9.82);
					linkage.getLoadData().setMass(lastWeight);

					isFinished = false;
					while (!isFinished) {

						 myforce = calcForce();
					
						double weight = Math.abs((myforce / 9.82));
						
						if (myforce < 0) {
							lastWeight = lastWeight + weight; 
							
							//logger.info("Robert: lastWeight: with force < 0 :" + lastWeight + ", weight : " + weight);
							
							if (lastWeight > 0 && lastWeight <= 20) {
								linkage.getLoadData().setMass(lastWeight);
								//logger.info("+ weight + : " + lastWeight);
							} else if (lastWeight > 20) {
								lastWeight = 20;
								//logger.info("+ weight+ : " + lastWeight); 
							}

						} else if (myforce > 0) {

							lastWeight = lastWeight - weight; 
							
							//logger.info("Robert: lastWeight with force > 0 : " + lastWeight + ", weight : " + weight);
							
							if (lastWeight > 0 && lastWeight <= 20) {
								linkage.getLoadData().setMass(lastWeight);
								//logger.info("- weight- : " + lastWeight);
							} else if (lastWeight < 0) {
								lastWeight = 0;
								//logger.info("- weight- : " + lastWeight);
							}

						}

						ThreadUtil.milliSleep(100);

						while (suspended) {
							ThreadUtil.milliSleep(100);

						}
					}
				}

			}).start();

		} catch (Exception e) {
			logger.info("Robert: thread is intruppted");
			e.printStackTrace();
		}
	}

	/**
	 * calculate the forces
	 * 
	 * @return
	 */
	private double calcForce() {

		ForceSensorData data = null;

		data = robot.getExternalForceTorque(tcp, World.Current.getRootFrame());

		if (data.isForceValid(20)) {
			Vector force = data.getForce();
			forceInZ = force.getZ();

			//ThreadUtil.milliSleep(100);

			return forceInZ;
		}
		 return forceInZ;
	}

	/**
	 * register torques and a new trajectory frames for each torque value
	 */
	private void register() {
		// monitor axes while moving up
		try {
			new Thread(new Runnable() {

				public void run() {

					register = new ArrayList<TorqueSensorData>();
					weights = new ArrayList<Double>();
					trajectory = new ArrayList<Frame>();

					

					isFinished = false;

					TorqueSensorData torques = robot.getExternalTorque();
					Frame position = robot.getCurrentCartesianPosition(linkage.getFrame("/RecTCP"));

					for (int i = 0; i < frameList.size(); i++) {

						while (frameList.get(i).distanceTo(robot.getCurrentCartesianPosition(linkage.getFrame("/RecTCP"))) >= 3) {

							torques = robot.getExternalTorque();
							position = robot.getCurrentCartesianPosition(linkage.getFrame("/RecTCP"));
							
							ThreadUtil.milliSleep(10);
							

							while (suspended) {
								ThreadUtil.milliSleep(100); 

							}
							
							if (isFinished) {
								return;
							}
						}
						
						//logger.info("registering data");
						weights.add(lastWeight);
						register.add(torques);
						trajectory.add(position);
					}

					logger.info("The new trajectory : " + trajectory.size());
				}
			}).start();

		} catch (Exception e) {
			logger.info("Robert: thread is intruppted");
			e.printStackTrace();
		}
	}

	/**
	 * get a new trajectory path during the 1st Route which has the same length
	 * as the weights array and torques array
	 * 
	 * @return {@link ArrayList} of frames
	 */
	public ArrayList<Frame> getPathFrames() {
		return trajectory;
	}

	/**
	 * get the torque values
	 * 
	 * @return
	 */
	public ArrayList<TorqueSensorData> getRegisteredTorques() {
		return register;
	}

	public ArrayList<Double> getWeights() {
		return weights;
	}

	/**
	 * stop calculating the weights and the torques
	 */
	public void stop() {
		isFinished = true;
		suspended = false;
	}

	public void suspend() {

		suspended = true;
	}

	public void resume() {

		suspended = false;
		// notifyAll();
	}

	/**
	 * check if the threads are running
	 * 
	 * @return
	 */
	public boolean isSuspended() {

		return suspended;

	}
}
