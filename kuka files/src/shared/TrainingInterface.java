package shared;

import java.util.ArrayList;

import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.roboticsAPI.motionModel.Spline;

public interface TrainingInterface {


	void moveMe();
	
	ArrayList<Frame> pathRecorder();

	ArrayList<Frame> trajectoryRecorder();

	void guiding() throws Exception;

	void resetData();

	boolean torqueRecorder();

	void end();
	
	void update(int cycle, int resistance);

	void pause();

	void resume();
	
	int play(int cycles, int resistance);
	
	void skipMotion();

	Spline getSwitchToPath(boolean isNextMotion);

	ArrayList<IMotion> calculateGuidedPath(ArrayList<Frame> frameList);
	
	ArrayList<Frame> filterTrack(ArrayList<Frame> trajectory, double minDistance);
	
	ArrayList<Double> velocityCalculation(ArrayList<Frame> trajectory, double minDistance);

	ArrayList<IMotion> calculateGuidedPathWithVelocities(ArrayList<Frame> frameList, ArrayList<Double> velarray);

	
}
