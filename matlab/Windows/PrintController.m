function [result] = PrintController(JointPath, FileName)
% This prints a controller in C in the Webots directory
% [result] = PrintController(JointPath, FileName)
%- JointPath: input Nxn vector. N is the amount of points to follow. n is the
%amount of joints of the manipulator.
%- Filename: name of the file containing the output code (without .c).
%- result: output string with status.

%Read begining and end of controller settings
fid_r1 = fopen('functions/init_c.txt','r');
s1 = fscanf(fid_r1,'%c');
fclose(fid_r1);
fid_r2 = fopen('functions/end_c.txt','r');
s2 = fscanf(fid_r2,'%c');
fclose(fid_r2);

%Print begining
address = strcat('webots_UR3e/controllers/UR3e_c/',FileName,'.c');
fid = fopen(address,'w');
%fid = fopen('webots_UR3e/controllers/UR3e_c/UR3etest2_c.c','w');
fprintf(fid,'%c',s1);
%Print joint path
fprintf(fid,'//Path and values\n');
size_JP = size(JointPath);
pointer_l = size_JP(1)-1;
fprintf(fid,'int pointer_l = %d; \n',pointer_l);

fprintf(fid,'double JointPath[%d1][6] = { ',size_JP(1));
for i=1:size_JP(1)
    a = JointPath(i,:);
    fprintf(fid,'{%6.4f, %6.4f, %6.4f, %6.4f, %6.4f, %6.4f}',a); 
    if i<size_JP(1)
        fprintf(fid,', \n');
    else
        fprintf(fid,'}; \n');
    end
end

%Print end
fprintf(fid,'%c',s2);
fclose(fid);
result='Code successfully print';
end