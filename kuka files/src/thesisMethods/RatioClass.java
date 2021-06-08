package thesisMethods;

import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Frame;

public class RatioClass {
	public boolean dLambda_Beta = false;
	
	public double[] x_old = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // format: {X, Y, Z, A, B, C}
	// controller parameters
	public final float KX_init = 200, KY_init = 200, KZ_init = 200, KROT_init = 50, DAll_init = 1;
	// for the main function
	public double[] Caxis = {0.0, 0.0, 0.0};
	public double Crot = 0.0, betaOut = 0.0;
	public double Dtotal = 0.0;
	
	// APF
	public static ApfClass APF = new ApfClass();
	// stiffness
	private double stiffness = 0.0;
	private double[] rho = {0.0,0.0,0.0,0.0};
	private double delta = 0.3;
	// damping
	final double deltaDX = 0.125;
	final double MAX_D = 1.0;
	final double MIN_D = 0.1;
	private double damping = 0.1;
	public double[] dMin = {0.0,0.0,0.0};
	public double[] dMax = {0.0,0.0,0.0};
	public double[] dx = {0.0,0.0,0.0};
	
	// to keep on track on the increments
	double dCaxis = 0.0, Caxis_old = 0.0, dDamping = 0.0, damping_old = 0.0 , dCangle = 0.0, Crot_old = 0.0;
	final double D_CAXIS = 10, D_DAMPING = 0.01;
	
	public double[] dummy = {0.0,0.0,0.0,0.0};
	double deltaT = 0.0;
	final double deltaT_max = 1000; 		// 1 sec
	//public void RatioClass(Frame fX_old)
	public final double DK_MAX = 500, DANGLE_MAX = 50, DD_MAX = 0.3;
	public void init(Frame fX_old, int tStep)
	{
    	//Frame fX_old = getFrame("/Recording/Center");
    	x_old[0] = fX_old.getX();
    	x_old[1] = fX_old.getY();
    	x_old[2] = fX_old.getZ();
    	x_old[3] = fX_old.getAlphaRad();
    	x_old[4] = fX_old.getBetaRad();
    	x_old[5] = fX_old.getGammaRad(); 
    	
    	deltaT = (double)tStep; // in ms
	}
	
	public boolean updateGains(double[] xVC, Frame fCenter, Frame fX, double temp_dC, double temp_dD)
	{
		//local variables
		//double[] xVC = {fVC.getX(), fVC.getY(),fVC.getZ()};
		double[] xCenter = {fCenter.getX(), fCenter.getY(),fCenter.getZ()};
		double[] x = {fX.getX(), fX.getY(),fX.getZ()};
		
		
    	// lambda ratio + stiffness on axis and angles
		//double deltaExp = 2*delta;
		dummy = APF.getStiffness(xCenter,xVC,x, delta, APF.axis);
		double stiffnessAxis = dummy[0];
		// evaluate if update is necesary
    	dCaxis = Math.abs(stiffnessAxis-Caxis_old);
    	if(dCaxis >= DK_MAX && stiffnessAxis > Caxis_old)
    	{
    		stiffnessAxis = Caxis_old + DK_MAX;
    	}
    	else if (dCaxis >= DK_MAX)
    	{
    		stiffnessAxis = Caxis_old - DK_MAX;
    	}
    	// check
    	if(stiffnessAxis > APF.MAX_K_AXIS)
    	{
    		stiffnessAxis = APF.MAX_K_AXIS;
    	}
    	else if (stiffnessAxis < APF.MIN_K_AXIS)
    	{
    		stiffnessAxis = APF.MIN_K_AXIS;
    	}
    	
		for (int j = 0; j<3; j++)
    	{
			Caxis[j] = stiffnessAxis;
    	}
		
		double[] dummy2= APF.getStiffness(xCenter,xVC,x, delta, APF.angle);		
    	
		Crot = dummy2[0];
		dCangle = Math.abs(Crot-Crot_old);
		if (dCangle >= DANGLE_MAX && damping > Crot_old)
    	{
			Crot = Crot_old + DANGLE_MAX;
    	}
    	else if (dCangle >= DANGLE_MAX)
    	{
    		Crot = Crot_old - DANGLE_MAX;
    	}
		// check
		if(Crot > APF.MAX_K_AXIS)
    	{
			Crot = APF.MAX_K_ANGLE;
    	}
    	else if (Crot < APF.MIN_K_ANGLE)
    	{
    		Crot = APF.MIN_K_ANGLE;
    	}
		
    	// beta ratio + damping 
		Dtotal = getDamping(xVC, fCenter, fX);    	
    	dDamping = Math.abs(damping-damping_old);
    	if (dDamping >= DD_MAX && damping > damping_old)
    	{
    		damping = damping_old + DD_MAX;
    	}
    	else if (dDamping >= DD_MAX)
    	{
    		damping = damping_old - DD_MAX;
    	}
    	// check
    	if(damping > MAX_D)
    	{
    		damping = MAX_D;
    	}
    	else if (damping < MIN_D)
    	{
    		damping = MIN_D;
    	}
    	
    	// final version
    	//dLambda_Beta = (dCaxis>=D_CAXIS)||(dDamping>=D_DAMPING);
    	// temporary version
    	dLambda_Beta = (dCaxis>=temp_dC*APF.MAX_K_AXIS)||(dDamping>=temp_dD*MAX_D);
    	
    	// update 
    	for(int i=0;i<x.length;i++)
    	{
    		x_old[i] = x[i];
    	}
    	Caxis_old = stiffnessAxis;
    	damping_old = damping;
    	
    	// end 		
		return dLambda_Beta;
	}
	// ---------------------------------------- Other APF methods --------------------------------------
    
    public double getDamping(double[] xVC, Frame fCenter, Frame fX)
	{
		//local variables
		//double[] xVC = {fVC.getX(), fVC.getY(),fVC.getZ()};
		double[] xCenter = {fCenter.getX(), fCenter.getY(),fCenter.getZ()};
		double[] x = {fX.getX(), fX.getY(),fX.getZ()};
		
	    // Calculate max, min and current velocity 
		for(int i=0;i<3;i++)
		{
			dMax[i] = Math.abs(xVC[i]-xCenter[i]);
			dx[i] = (1000/deltaT)*(Math.abs(x[i]-x_old[i]));
		}		
		    
		betaOut = APF.getLambda(dMin,dMax,dx,deltaDX);
	    
	    double localDamping = (MAX_D-MIN_D)*betaOut + MIN_D;
	    
	    return localDamping;
    }// public double getDamping
    
}
