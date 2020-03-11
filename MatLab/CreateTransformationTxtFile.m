%% Creating Transformation file for Kinect collection
% This creates the correct transformation for a collection where the master
% and sub2 are collected in a NFOV and sub1 and sub3 are collected in WFOV.
% If the depth of any of these cameras changes the transformations will
% need to be updated correctly.

clc; clear all ;close all
angle=1.3*(pi/180);
WFOV_NFOVTrans=[1 0 0 0;
    0 cos(angle) -sin(angle)  0;
    0 sin(angle) cos(angle)  0;
    0 0 0 1];

TransMaster=WFOV_NFOVTrans;

TransSub1Master=[0.003262548940 0.158023983240 -0.987429857254 2240.754882812500;
-0.134737223387 0.978500425816 0.156149774790 -326.543151855469;
0.990876019001 0.132534116507 0.024484118447 2489.085449218750;
0.000000000000 0.000000000000 0.000000000000 1.000000000000];

TransSub2Master=[ -0.999287605286 0.018243510276 -0.033037651330 117.151992797852;
0.009763699956 0.970568001270 0.240629211068 -600.226440429688;
0.036455206573 0.240135207772 -0.970054686069 5110.304199218750;
0.000000000000 0.000000000000 0.000000000000 1.000000000000];

TransSub2Master=TransSub2Master*WFOV_NFOVTrans;

TransSub3Master=[0.007361589931 -0.138939142227 0.990273535252 -2110.854736328125;
0.119770132005 0.983294010162 0.137069523335 -259.883605957031;
-0.992774367332 0.117596141994 0.023879365996 2609.124267578125;
0.000000000000 0.000000000000 0.000000000000 1.000000000000];

%% Writing Text File
fid=fopen('C:\Users\Abby.Eustace\Desktop\Kinect Azure\CalFeb19Transv2.txt','wt'); % Use for batch process
fprintf(fid,'###\nMaster.mkv');    
fprintf(fid,'\n%4.12f\t%4.12f\t%4.12f\t%4.12f',TransMaster');
fprintf(fid,'\n###\nSub1.mkv');    
fprintf(fid,'\n%4.12f\t%4.12f\t%4.12f\t%4.12f',TransSub1Master');
fprintf(fid,'\n###\nSub2.mkv');    
fprintf(fid,'\n%4.12f\t%4.12f\t%4.12f\t%4.12f',TransSub2Master');
fprintf(fid,'\n###\nSub3.mkv');    
fprintf(fid,'\n%4.12f\t%4.12f\t%4.12f\t%4.12f',TransSub3Master');
fclose(fid);
