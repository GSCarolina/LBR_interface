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

public class VClass {
	public double tubeRadius = 250; 	// mm
	public final int NONE_TYPE = 0, WALL_TYPE = 1, OTHER_TYPE = 2;
	public final int X_AXIS = 0, Y_AXIS = 1, Z_AXIS = 2;
	// setting virtual constraint wall
    public double[] setVCfromCoppelia(Frame currentX, int type, double[] vcPoint)
    {
    	double[] simVC = {0.0,0.0,0.0};
    	
    	switch(type)
    	{
    	case WALL_TYPE:
    		simVC[0] = currentX.getX();
    		simVC[1] = currentX.getY();
    		simVC[2] = vcPoint[2];
    	default:
    		simVC[0] = currentX.getX();
    		simVC[1] = currentX.getY();
    		simVC[2] = vcPoint[2];
    	}
    	
    	return simVC;
    }
    
    // setting virtual constraint tube
    public double[] setVCtube(Frame fCenter, Frame currentX, int axis)
    {
    	double[] xCenter = {fCenter.getX(),fCenter.getY(),fCenter.getZ()};
    	double[] x = {currentX.getX(),currentX.getY(),currentX.getZ()};
    	double[] tubeVC = {0.0,0.0,0.0};
    	
    	// Calculate radius and angle
    	double[] XYZ_pow = {Math.pow(xCenter[0]-x[0], 2), Math.pow(xCenter[1]-x[1], 2), Math.pow(xCenter[2]-x[2], 2)};
    	//double rYZ = Math.sqrt(XYZ_pow[1] + XYZ_pow[2]);
    	
    	double[] rXYZ = {Math.sqrt(XYZ_pow[1] + XYZ_pow[2]), Math.sqrt(XYZ_pow[0] + XYZ_pow[2]), Math.sqrt(XYZ_pow[0] + XYZ_pow[1])};
    	
    	double cosA = 0.0, senA = 1.0;
    	
    	boolean valid_axis = (axis>=X_AXIS) && (axis<=Z_AXIS);
    	

    	//default value (if its not valid, it will still return a value)
    	tubeVC[0] = xCenter[0];
    	tubeVC[1] = xCenter[1] + tubeRadius*cosA;
    	tubeVC[2] = xCenter[2] + tubeRadius*senA;
    	
    	// actual value    	
    	if(valid_axis)
    	{
    		// this condition can only be evaluated AFTER the previous one
    		if(rXYZ[axis]>0.0001)
    		{
    			cosA = (x[1]-xCenter[1])/rXYZ[axis];
        		senA = (x[2]-xCenter[2])/rXYZ[axis];
    			switch(axis)
    			{
    			case X_AXIS:
    				cosA = (x[1]-xCenter[1])/rXYZ[axis];
            		senA = (x[2]-xCenter[2])/rXYZ[axis];
            		tubeVC[0] = xCenter[0];
                	tubeVC[1] = xCenter[1] + tubeRadius*cosA;
                	tubeVC[2] = xCenter[2] + tubeRadius*senA;
    				break;
    				
    			case Y_AXIS:
    				cosA = (x[0]-xCenter[0])/rXYZ[axis];
            		senA = (x[2]-xCenter[2])/rXYZ[axis];
            		tubeVC[0] = xCenter[0]+ tubeRadius*cosA;
                	tubeVC[1] = xCenter[1]; 
                	tubeVC[2] = xCenter[2] + tubeRadius*senA;
    				break;
    				
    			case Z_AXIS:
    				cosA = (x[0]-xCenter[0])/rXYZ[axis];
            		senA = (x[1]-xCenter[1])/rXYZ[axis];
            		tubeVC[0] = xCenter[0] + tubeRadius*cosA;
                	tubeVC[1] = xCenter[1] + tubeRadius*senA;
                	tubeVC[2] = xCenter[2];
    				break;
    			
    			}

    		}
    	}
    	

    	
    	return tubeVC;
    }

    
    public double[] selectVC(Frame fCenter, Frame currentX, int axis,int type, double[] vcPoint)
    {
        double[] vcOut = {0.0,0.0,0.0};
        
        double[] copPoint = setVCfromCoppelia(currentX, type, vcPoint);
        double[] tubePoint = setVCtube(fCenter, currentX, axis);
        
        double[] x = {currentX.getX(),currentX.getY(),currentX.getZ()};
        
        // check which object is closer to the current position
    	double[] x_sim = {Math.pow(copPoint[0]-x[0], 2), Math.pow(copPoint[1]-x[1], 2), Math.pow(copPoint[2]-x[2], 2)};  
    	double x_sim_distance = Math.sqrt(x_sim[0] + x_sim[1] + x_sim[2]);
        
    	double[] x_tube = {Math.pow(tubePoint[0]-x[0], 2), Math.pow(tubePoint[1]-x[1], 2), Math.pow(tubePoint[2]-x[2], 2)};  
    	double x_tube_distance = Math.sqrt(x_tube[0] + x_tube[1] + x_tube[2]);
        
    	if(x_tube_distance>=x_sim_distance)
    	{
    		for(int i=0;i<3;i++)
    		{
    			vcOut[i] = tubePoint[i];
    		}
    	}
    	else
    	{
    		for(int i=0;i<3;i++)
    		{
    			vcOut[i] = copPoint[i];
    		}
    	}
    	
        return vcOut;    	
    }

}
