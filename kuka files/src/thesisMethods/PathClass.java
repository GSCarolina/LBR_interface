package thesisMethods;

import java.util.ArrayList;
import java.util.List;

import thesisUtilities.*;

import com.kuka.roboticsAPI.deviceModel.Device;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
import com.kuka.task.ITaskLogger;

public class PathClass {
	// Local variables
	TorqueSensorData measuredData, externalData;
	public double[] qA = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	public double[] qA_old = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double diffqA = 0.0;
	
	public final double[] qA_diff = {0.2, 0.075, 0.2, 0.075, 0.075, 0.075, 0.075}; // max difference to consider the robot being misplaced
	public double[] PEE = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // format: {X, Y, Z, A, B, C}
	
	public double[] measuredTorques, externalTorques;
	public double[] tA = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	public double[] tA_old = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	public double[] tAext = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	public double[] tAext_old = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	public double[] tA_diff = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	public double[] tAext_diff = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	public double[] teeA = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	public double[] fEE = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // format: {X, Y, Z, A, B, C}
	final double LIMIT = 10;	// Constant value
	
	// recording paths using hand-guiding
	public List<JointPosition> recordedPathJS, correctedPathJS; 
	public List<Frame> recordedPathCS; 
	// unused
	public List<TorqueSensorData>  recordedTorqueJS, recordedExtJS;
	public List<ForceSensorData>  recordedForce;
	
	public JointPosition _jointPos, _jointPos_old;	
	
	
	// Files handling 
	public WriteToFile jointFile;
	public WriteToFile2 posFile;
	public WriteToFile3 forFile;
	public ReadFile recording;
	public String message, jointName, posName;
	public String forName;
	public String recName;
	
	public String screenMsg = new String ();
	
	public void init()
	{
		// trajectories start up
		recordedPathJS = new ArrayList<JointPosition>();
		recordedPathCS = new ArrayList<Frame>();
		correctedPathJS = new ArrayList<JointPosition>();
		//recordedPathJST = new ArrayList<JointPosition>();
		recordedTorqueJS = new ArrayList<TorqueSensorData>();
		recordedExtJS = new ArrayList<TorqueSensorData>();
		recordedForce  = new ArrayList<ForceSensorData>();
		
    	recordedPathJS.clear();
    	correctedPathJS.clear();
    	recordedPathCS.clear();
	}
	
	public void loadTrajectory(String exName)
    {
		boolean errorLoading = false;
		boolean errorOpening = false;
    	// load recording
    	recording = new ReadFile();        
    	recName = "C:\\Users\\KukaUser\\Downloads\\thesisData\\pos\\JS_" +exName+".txt";
    	recording.init(recName); 
    	recording.readContent();
    	
    	if(recording.error)
    	{
    		errorLoading = true;
    	}
 	
    	recordedPathJS.clear();
    	recordedPathCS.clear();
    	
        // process data loop
    	String del = new String(",");
        String[] dataLine;
        int dataNr = 0;
        for(int i=0; i<recording.linesNr;i++){
            dataLine = recording.splitData(del,i);
            dataNr = dataLine.length;
            for(int j=0;j<dataNr;j++){
                qA[j] = Double.parseDouble(dataLine[j]);
            }
            
            if(dataNr == 7)
            {
            	_jointPos = new JointPosition(qA[0],qA[1],qA[2],qA[3],qA[4],qA[5],qA[6]);
                recordedPathJS.add(_jointPos);
            }
            
        }
        
    	if(!errorLoading||dataNr!=6)
    	{
    		screenMsg = "Trajectory sucessfully loaded with " + recording.linesNr + " points";
    	}
    	else if (errorLoading)
    	{
    		screenMsg = "Error while reading or opening file";
    	}
    	else
    	{
    		screenMsg = "Error on the path parameters";
    	}
    	
        recording.end();
        
    }
	
	public void recordTrajectory(JointPosition _jointCurrent, Frame current, TorqueSensorData measuredData, TorqueSensorData externalData)
	{
    	// local variables
    	boolean stopRobot = false, updateFile = false;
    	double velJS = 0.3;
    	int updateCnt = 0;
    	
		// Joint-space
		//_jointPos = _lbr.getCurrentJointPosition();
		qA[0] = _jointCurrent.get(JointEnum.J1);
		qA[1] = _jointCurrent.get(JointEnum.J2);
		qA[2] = _jointCurrent.get(JointEnum.J3);
		qA[3] = _jointCurrent.get(JointEnum.J4);
		qA[4] = _jointCurrent.get(JointEnum.J5);
		qA[5] = _jointCurrent.get(JointEnum.J6);
		qA[6] = _jointCurrent.get(JointEnum.J7);
		
		// a new position has been reached if at least 1 joint has been moved 0.075 rad
		for(int j=0;j<7;j++)
		{
			diffqA = Math.abs(qA[j]-qA_old[j]);
			if(diffqA >= 0.075)
			{
				updateFile = true;
			}
		}
		// If a new position has been reached, add it to the recordings
		if(updateFile)
		{
			//getLogger().info("Updating path nr. " + updateCnt);
			// Joint-space
			recordedPathJS.add(_jointPos);
			// Cartesian-space
			//current = _lbr.getCurrentCartesianPosition(_linkage.getFrame("/TCP"), World.Current.getRootFrame());
			//current = _lbr.getCurrentCartesianPosition(_linkage.getFrame("/TCP"));
			recordedPathCS.add(current);
			
			// update joints values
			for(int j=0;j<7;j++)
			{
				qA_old[j] = qA[j];
			}
			
			// save torques as well
			//measuredData = _lbr.getMeasuredTorque();
			recordedTorqueJS.add(measuredData);
			
			//externalData = _lbr.getExternalTorque();
			recordedExtJS.add(externalData);				
			//
			updateCnt++;
		}// if(updateFile)
	} // recordTrajectory
	
	public void startPrintTrajectory(String exName)
	{
    	jointName = "C:\\Users\\KukaUser\\Downloads\\thesisData\\pos\\JS_" +exName+".txt";
    	jointFile.init(jointName);
    	
    	posName = "C:\\Users\\KukaUser\\Downloads\\thesisData\\pos\\CS_" + exName + ".txt";    	
    	posFile.init(posName);
    	
    	forName = "C:\\Users\\KukaUser\\Downloads\\thesisData\\pos\\FORCE_" +exName+".txt";    	
    	forFile.init(forName);    	
	}
	
	public void endPrintTrajectory(String exName)
	{
		jointFile.end();
		posFile.end();
		forFile.end();
	}
	
	public void printTrajectory(int index, ITaskLogger _logger)
	{
		_logger.info("Printing");
		Frame current;
    	// Saving Joint-space path
   	
    	_logger.info(jointName);
    	
		
    	String qMsg = new String();
    	
    	_logger.info("size " + recordedPathJS.size());
    	
    	
		// Joint-Space
		_jointPos = recordedPathJS.get(index);
		_logger.info("accessing joint path point nr" + index );
		qA[0] = _jointPos.get(JointEnum.J1);
		_logger.info("joint 0");
		qA[1] = _jointPos.get(JointEnum.J1);
		_logger.info("joint 1");
		qA[2] = _jointPos.get(JointEnum.J1);
		qA[3] = _jointPos.get(JointEnum.J1);
		qA[4] = _jointPos.get(JointEnum.J1);
		qA[5] = _jointPos.get(JointEnum.J1);
		qA[6] = _jointPos.get(JointEnum.J1);
		_logger.info("joint 6");
		_logger.info("setting string for printing");
		qMsg = new String(qA[0]+","+qA[1]+","+qA[2]+","+qA[3]+","+qA[4]+","+qA[5]+","+qA[6]);
		
		//jointFile.doStuff(writeFile,qMsg);
		_logger.info("Process");
		jointFile.saveLine(qMsg);
		
    	
    	// Saving Cartesian-space path
    	current = recordedPathCS.get(index);
		PEE[0] = current.getX();
		PEE[1] = current.getY();
		PEE[2] = current.getZ();
		PEE[3] = current.getAlphaRad();
		PEE[4] = current.getBetaRad();
		PEE[5] = current.getGammaRad();
		qMsg = new String(PEE[0]+","+PEE[1]+","+PEE[2]+","+PEE[3]+","+PEE[4]+","+PEE[5]);
		
		posFile.saveLine(qMsg);
		
		
    	// Saving forces
    	String tMsg = new String(), tExtMsg = new String();   	
		// From the whole body 
		measuredData = recordedTorqueJS.get(index);		
		externalData = recordedExtJS.get(index);
		
		// Joint torques
		tA[0] = measuredData.getSingleTorqueValue(JointEnum.J1);
		tA[1] = measuredData.getSingleTorqueValue(JointEnum.J2);
		tA[2] = measuredData.getSingleTorqueValue(JointEnum.J3);
		tA[3] = measuredData.getSingleTorqueValue(JointEnum.J4);
		tA[4] = measuredData.getSingleTorqueValue(JointEnum.J5);
		tA[5] = measuredData.getSingleTorqueValue(JointEnum.J6);
		tA[6] = measuredData.getSingleTorqueValue(JointEnum.J7);
		tMsg = new String(tA[0]+","+tA[1]+","+tA[2]+","+tA[3]+","+tA[4]+","+tA[5]+ "," + tA[6] + ";");
		
		tAext[0] = externalData.getSingleTorqueValue(JointEnum.J1);
		tAext[1] = externalData.getSingleTorqueValue(JointEnum.J2);
		tAext[2] = externalData.getSingleTorqueValue(JointEnum.J3);
		tAext[3] = externalData.getSingleTorqueValue(JointEnum.J4);
		tAext[4] = externalData.getSingleTorqueValue(JointEnum.J5);
		tAext[5] = externalData.getSingleTorqueValue(JointEnum.J6);
		tAext[6] = externalData.getSingleTorqueValue(JointEnum.J7);
		
		tExtMsg = new String(tAext[0]+","+tAext[1]+","+tAext[2]+","+tAext[3]+","+tAext[4]+","+tAext[5]+ "," + tAext[6] + ";");
		
		qMsg = tMsg + tExtMsg;
		posFile.saveLine(qMsg);	
		
	}
	
	public boolean updateTrajectory(JointPosition _jointCurrent, int index)
	{
		boolean updatePath = false;
		_jointPos = recordedPathJS.get(index);
		qA_old[0] = _jointPos.get(JointEnum.J1);
		qA_old[1] = _jointPos.get(JointEnum.J2);
		qA_old[2] = _jointPos.get(JointEnum.J3);
		qA_old[3] = _jointPos.get(JointEnum.J4);
		qA_old[4] = _jointPos.get(JointEnum.J5);
		qA_old[5] = _jointPos.get(JointEnum.J6);
		qA_old[6] = _jointPos.get(JointEnum.J7);  
		
		qA[0] = _jointCurrent.get(JointEnum.J1);
		qA[1] = _jointCurrent.get(JointEnum.J2);
		qA[2] = _jointCurrent.get(JointEnum.J3);
		qA[3] = _jointCurrent.get(JointEnum.J4);
		qA[4] = _jointCurrent.get(JointEnum.J5);
		qA[5] = _jointCurrent.get(JointEnum.J6);
		qA[6] = _jointCurrent.get(JointEnum.J7); 
		
		for(int j=0;j<7;j++)
		{
			diffqA = Math.abs(qA[j]-qA_old[j]);
			if(diffqA >= qA_diff[j])
			{
				updatePath = true;
		    	//getLogger().info("New position");
		    	
			}
		}
		if(updatePath){
			// udpate path
			correctedPathJS.add(_jointPos);
			//ThreadUtil.milliSleep(1000);
		}
		else
		{
			// keep path
			correctedPathJS.add(recordedPathJS.get(index));
		}
		return updatePath;
	}//updateTrajectory
	
}
