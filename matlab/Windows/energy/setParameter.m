function [parameter] = setParameter(energy,maxValue, minValue)
%Function that returns a spring parameter for the impedance controller.
%It can be used for both the stiffness and the damping.
% [parameter] = setParameter(energy,maxValue, minValue)
% - energy: value [0,1] from the Artificial Potential Field.
% - maxValue: maximum value for the stiffness/damping.
% - minValue: minimum value for the stiffness/damping.
%Example:
% K_stiffness = setParameter(0.5,5000, 100);

valid = 1;
temp_output = 0;

   if (minValue <=0)
       valid = 0;
       disp('Invalid minimum value.');
   end

   if (maxValue <=0)
     valid = 0;
     disp('Invalid maximum value.');
   end
   
   if (energy >1)
     valid = 0;
     disp('Energy value above 1 will return the maximum value.');
     temp_output = maxValue;
   elseif (energy<0)
     valid = 0;
     disp('Energy value below 0 will return the minimum value.');
     temp_output = minValue;
   end
   
   if (valid == 1)
       c = 0.5 * (log(maxValue/minValue)-1);
       eta = 2 * minValue * exp(-c^2);
       temp_output = 0.5 * eta * exp((energy+c)^2);
   end
   
parameter = temp_output;
end

