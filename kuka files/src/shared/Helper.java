package shared;

import java.util.ArrayList;

import com.kuka.roboticsAPI.geometricModel.Frame;

public class Helper extends Training{

	@Override
	public void moveMe() {
		guiding();
	}

	@Override
	public ArrayList<Frame> pathRecorder() {
		return super.pathRecorder();
	}
	
	
	@Override
	public ArrayList<Frame> filterTrack(ArrayList<Frame> trajectory, double minDistance) {
		return super.filterTrack(trajectory, minDistance);
	}
	
	@Override
	public ArrayList<Double> velocityCalculation(ArrayList<Frame> trajectory, double minDistance) {
		return super.velocityCalculation(trajectory, minDistance);
	}
	
}
