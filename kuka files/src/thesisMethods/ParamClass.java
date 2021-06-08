/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package thesisMethods;

//import java.lang.*;

/**
 *
 * @author Carolina
 */
public class ParamClass {
    final double MAX_E = 1.0;
    final double MIN_E = 0.0;
    // Constant values for the C-axis stiffness
    final double MAX_K_AXIS = 5000;
    final double MIN_K_AXIS = 200;
    final double CK_AXIS = 1.1094;
    final double ETA_AXIS = 116.8174;
    // Constant values for the C-angles stiffness
    final double MAX_K_ANGLE = 300;
    final double MIN_K_ANGLE = 10;
    final double CK_ANGLE = 1.2006;
    final double ETA_ANGLE = 4.7317;
    // Constant values for the Damping
    final double MAX_D = 1.0;
    final double MIN_D = 0.1;
    final double C_D = 1.8026;
    final double ETA_D = 0.00077602;

    public double getKAxis(double energy){
        double output = 0.0;
        if(energy > MAX_E){
            output = MAX_K_AXIS; 
        }
        else if (energy < MIN_E)
        {
            output = MIN_K_AXIS;
        }
        else{
            double temp_gain = 0.5*ETA_AXIS;
            double square = Math.pow((energy + CK_AXIS), 2);
            output = temp_gain * Math.exp(square);
        }
        return output;
    }
    
    public double getKAngle(double energy){
        double output = 0.0;
        if(energy > MAX_E){
            output = MAX_K_ANGLE; 
        }
        else if (energy < MIN_E)
        {
            output = MIN_K_ANGLE;
        }
        else{
            double temp_gain = 0.5*ETA_ANGLE;
            double square = Math.pow((energy + CK_ANGLE), 2);
            output = temp_gain * Math.exp(square);
        }
        return output;
    }
    
    public double getDamp(double energy){
        double output = 0.0;
        if(energy > MAX_E){
            output = MAX_D; 
        }
        else if (energy < MIN_E)
        {
            output = MIN_D;
        }
        else{
            double temp_gain = 0.5*ETA_D;
            double square = Math.pow((energy + C_D), 2);
            output = temp_gain * Math.exp(square);
        }
        return output;
    }
}

