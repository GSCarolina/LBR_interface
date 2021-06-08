package thesisMethods;

/**
*
* @author Carolina
*/
public class ApfClass {
   // Artificial Potential Field class, with ratios and gains calculation
   // ---------------- Fields ----------------
   final double MAX_LAMBDA = 1.0;
   final double MIN_LAMBDA = 0.0;
   // Constant values for the C-axis stiffness
   final double MAX_K_AXIS = 5000;
   final double MIN_K_AXIS = 200;
   // Constant values for the C-angles stiffness
   final double MAX_K_ANGLE = 300;
   final double MIN_K_ANGLE = 10;
   
   public final boolean axis = true, angle = false;
   
   public double rho0, rho1, rho, lambdaOut = 0;
   // ---------------- Methods ----------------
   public double[] calculateRho(double[] xCenter, double[] xWall,double[] x, double delta)
   {
       // Calculate distances 
       double xtemp_max = 0.0, xtemp = 0.0, xmax_acc = 0.0, x_acc = 0.0, x1_acc = 0.0;
       for (int i=0;i<xCenter.length;i++)
       {
           // rho0
           xtemp_max = (xWall[i]-xCenter[i])*(xWall[i]-xCenter[i]);
           xmax_acc = xmax_acc + xtemp_max;    
           // rho1
           xtemp = (x[i]-xCenter[i])*(x[i]-xCenter[i]);
           x1_acc = x1_acc + xtemp;    
           // rho
           xtemp = (xWall[i]-x[i])*(xWall[i]-x[i]);
           x_acc = x_acc + xtemp;
       }

       // Calculate parameters       
       double[] rhoOut = {Math.sqrt(xmax_acc), Math.sqrt(x1_acc), Math.sqrt(x_acc)};
       return rhoOut;
   }
   
   public double[] getCorrectedPoint(double[] xCenter, double[] xWall,double[] x, double delta)
   {
   // Function that corrects a point within an allowed workspace
   // double[] xCorrected = getCorrectedPoint(xCenter,xWall,x,delta)
   // - xCenter = centroid location vector [x y z]
   // - xWall = virtual wall closest point vector [x y z] 
   // - x = point [x y z]
   // - delta = distance offset (0,1) close to a wall, being 0 allowing to reach
   // the virtual constraint and 1 not allowing any displacement at all
   
   // Calculate parameters    
   double[] rhoTemp = calculateRho(xCenter, xWall,x,delta);
   rho0 = rhoTemp[0];
   rho1 = rhoTemp[1];
   rho = rhoTemp[2];
   
   double[] xout = new double[x.length];
   for(int i=0;i<xout.length;i++)
   {
       xout[i] = x[i];
   }
   
    // Rebounce point if out of limits 
   if (rho<=delta*rho0 || (rho1>=rho0))
   {
       
       if(x.length==1) // 1D
       {
           xout[0] = xCenter[0] + delta*rho0;
       }

       if(x.length>=2) //2 and 3D
       {
           // Calculate angle between center and x
           //cosC = (x[1]-xCenter[1])/sqrt((x[1]-xCenter[1])^2 + (x[2]-xCenter[2])^2);
           //senC = (x[2]-xCenter[2])/sqrt((x[1]-xCenter[1])^2 + (x[2]-xCenter[2])^2);
           //C = acos(cosC);

           // Calculate angle between center and vc
           double cosC = 0, senC = 0;
           
           if(xWall[0] != xCenter[0] && xWall[1] != xCenter[1])
           {
               double temp1 = Math.pow(xWall[0]-xCenter[0],2) + Math.pow(xWall[1]-xCenter[1],2);
               cosC = (xWall[0]-xCenter[0])/Math.sqrt(temp1);
               senC = (xWall[1]-xCenter[1])/Math.sqrt(temp1);
           }

           xout[0] = xCenter[0] + (1-delta)*rho0*cosC;
           xout[1] = xCenter[1] + (1-delta)*rho0*senC;
       }

       if(x.length==3) // 3D
       {
           double temp3 = Math.pow(xWall[0]-xCenter[0],2) + Math.pow(xWall[2]-xCenter[2],2);
           double senA = (xWall[2]-xCenter[2])/Math.sqrt(temp3);
           //A = acos(cosA);
           xout[2] = xCenter[2] + (1-delta)*rho0*senA;
       
       }        
   }       
   
   return xout;
   }// public double getCorrectedPoint

   public double getLambda(double[] xCenter, double[] xWall,double[] x, double delta)
   {
   // Function that calculates the potential ratio lambda of a point towards a wall
   // double lambda = getLambda(xCenter,xWall,x,delta)
   // - xCenter = centroid location vector [x y z]
   // - xWall = virtual wall closest point vector [x y z] 
   // - x = point [x y z]
   // - delta = distance offset (0,1) close to a wall, being 0 allowing to reach
   // the virtual constraint and 1 not allowing any displacement at all
   // - lambda = potential ratio from 0 to 1

   // Calculate parameters    
   double[] rhoTemp = calculateRho(xCenter, xWall,x,delta);
   rho0 = rhoTemp[0];
   rho1 = rhoTemp[1];
   rho = rhoTemp[2];
   
   double eta = 2*(delta*delta)*(rho0*rho0)/(1+(delta*delta)-2*delta);

   double lambda = 0.0;
   
    // Rebounce point if out of limits 
   if (rho<=delta*rho0 || (rho1>=rho0))
   {
       lambda = MAX_LAMBDA;     
   }   
   else
   {
       lambda = 0.5*eta*Math.pow((1/rho)-(1/rho0),2);
   } 
   
   if(lambda < MIN_LAMBDA)
   {
       lambda = MIN_LAMBDA;
   }
       
   return lambda;
   }// public double getLambda
   
   public double[] getStiffness(double[] xCenter, double[] xWall,double[] x, double delta, boolean type)
   {
   // Function that calculates the potential ratio of a point towards a wall
   // double gain = getPotentialRatio(xCenter,xWall,x,delta,type)
   // - xCenter = centroid location vector [x y z]
   // - xWall = virtual wall closest point vector [x y z] 
   // - x = point [x y z]
   // - delta = distance offset (0,1) close to a wall, being 0 allowing to reach
   // the virtual constraint and 1 not allowing any displacement at all
   // - type = true for CS axis, false for CS angles
   // - gain = stiffness gain from min to max value (may vary depending on the 
   // robot model)

   // Calculate parameters    
   lambdaOut = getLambda(xCenter,xWall,x,delta);   
   double kmax = MAX_K_ANGLE, kmin = MIN_K_ANGLE;
   // Gain
   if(type)
   {
       kmax = MAX_K_AXIS;
       kmin = MIN_K_AXIS;
   }
   double stiffness = (kmax - kmin)*lambdaOut + kmin;
   
   if(stiffness > kmax)
   {
       stiffness = kmax;
   }
   else if (stiffness < kmin)
   {
       stiffness = kmin;
   }
   
   //return stiffness;
   double[] dumOut = {stiffness, rho0, rho1, rho};
   return dumOut;
   }// public double getStiffness
}
