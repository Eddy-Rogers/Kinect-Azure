%% Creating Transformation file for Kinect collection
% This creates the correct transformation for a collection where the master
% and sub2 are collected in a NFOV and sub1 and sub3 are collected in WFOV.
% If the depth of any of these cameras changes the transformations will
% need to be updated correctly.

clc; clear all ;close all


TransSubMaster=[0.135774418712 0.116779394448 -0.983833253384 1401.071289062500;
-0.132871106267 0.986204266548 0.098723888397 -90.831909179688;
0.981789469719 0.117318831384 0.149417921901 968.514892578125;
0.000000000000 0.000000000000 0.000000000000 1.000000000000];


%% Writing Text File
fid=fopen('D:\DUPelv\Cal(Mar5)\CalMar5.txt','wt'); 
fprintf(fid,'\n###\nSub.mkv');    
fprintf(fid,'\n%4.12f\t%4.12f\t%4.12f\t%4.12f',TransSubMaster');
fclose(fid);
