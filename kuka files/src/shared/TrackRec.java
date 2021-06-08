package shared;

import java.util.ArrayList;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplicationData;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;

public class TrackRec extends Thread {
	private LBR robot;
	private Tool linkage;
	private ArrayList<Frame> trackFrames;
	private RoboticsAPIApplicationData roboticApiAppData;


	public TrackRec(LBR _robot, Tool _linkage, IApplicationData appData) {
		robot = _robot;
		linkage = _linkage;
		roboticApiAppData = (RoboticsAPIApplicationData) appData;
		trackFrames = new ArrayList<Frame>();

	}

	@Override
	public void run() {
		do {
			trackFrames.add(robot.getCurrentCartesianPosition(linkage.getFrame("/RecTCP")));
			ThreadUtil.milliSleep(100);
		}while (roboticApiAppData.getProcessData("trackPath").getValue());

	}

	public ArrayList<Frame> getList() {

		return trackFrames;
	}
	
}
