package thesisMethods;

import com.kuka.roboticsAPI.geometricModel.math.Vector;

public class VelClass {
	// Internal constants
	public final static double maxVel = 0.5, minVel = 0.00001;
	public final static double maxForce = 27, minForce = 20;
	public final static double m = (maxVel-minVel)/(Math.exp(maxForce-minForce)-1);
	public final static double n = minVel - m;
	
	public final static double DVEL_JS_MAX = 0.75;
	
	public static double velJS = minVel;
	
	// Internal variables
	static int fCheck = 0, updateCheclLimit = 0;
	public static boolean updateVel = false ;
	public static boolean updateCheck = false;
	
	public static double fX, fY, fZ, fCS, tempCS, velJS_old, velJS_temp, velJS_diff, fCS_old, f_temp, f_diff;
	public final static double fLimit = 5;
	public static final double vLimit = 0.1;
	
	public VelClass()
	{
		fCS_old = 0.0;
		velJS_old = 0.0;
		updateVel = false;
		velJS = minVel;
		updateCheclLimit = 20;
	}
	
    public static double getVelocity(double force)
    {
    	
    	double dForce = force - minForce;
    	if(force < minForce)
    	{
    		dForce = 0;
    	}
    	else if (force > maxForce)
    	{
    		dForce = maxForce-minForce;
    	}
    	    	
    	double output = n + (m*Math.exp(dForce));
    	
    	if (output<minVel)
    	{
    		output = minVel;
    	}
    	else if(output > maxVel)
    	{
    		output = maxVel;
    	}
    	
    	return output;

    }
    
    public static boolean updateVelocity(Vector forceVector)
    {
    	updateVel = false;
    	fX = forceVector.getX();
		fY = forceVector.getY();
		fZ = forceVector.getZ();
		
		tempCS = Math.pow(fX,2)+Math.pow(fY,2)+Math.pow(fZ,2);
		fCS = Math.sqrt(tempCS);
		
		//updateVel = (fCS >= fLimit);				
		velJS_temp = getVelocity(fCS);
		
		// update velocity value
		//velJS_diff = Math.abs(velJS_temp -  velJS);
		f_diff = Math.abs(fCS - fCS_old);
		//f_diff = Math.abs(fCS);
		
		velJS_diff = Math.abs(velJS_temp - velJS_old);
		
		// update condition
		updateCheck = fCheck>updateCheclLimit;
		updateVel = (updateCheck || velJS_diff>=vLimit);
		
		if(updateVel)
		{
			// limit how much it can increase at once
			if(velJS_diff > DVEL_JS_MAX && velJS_temp > velJS_old)
			{
				velJS_temp = velJS_old + DVEL_JS_MAX;
			}
			else if (velJS_diff > DVEL_JS_MAX)
			{
				velJS_temp = velJS_old - DVEL_JS_MAX;
			}
			//velJS_old = velJS;
			fCS_old = fCS;
			velJS_old = velJS;
			velJS = velJS_temp;
			fCheck = 0;
		}
		else
		{
			fCheck++;
		}
		
		return updateVel;
    }
}
