%% Creating Transformation file for Kinect collection
% This creates the correct transformation for a collection where the master
% and sub2 are collected in a NFOV and sub1 and sub3 are collected in WFOV.
% If the depth of any of these cameras changes the transformations will
% need to be updated correctly.

clc; clear all ;close all
angle=1.3*(pi/180);
% % % WFOV_NFOVTrans=[1 0 0 0;
% % %     0 cos(angle) -sin(angle)  0;
% % %     0 sin(angle) cos(angle)  0;
% % %     0 0 0 1];
% % % 
% % % TransMaster=WFOV_NFOVTrans;

TransMaster=[1 0 0 0;
            0 1 0 0;
            0 0 1 0;
            0 0 0 1];

TransSub1Master=[-0.020742300898 0.319888502359 -0.947228133678 1215.772338867188;
-0.248310670257 0.916097521782 0.314812868834 -456.498535156250;
0.968458354473 0.241736799479 0.060429759324 1968.369628906250;
0.000000000000 0.000000000000 0.000000000000 1.000000000000];

TransSub2Master=[ -0.998221755028 0.028481593356 0.052365597337 -91.296363830566;
0.048391316086 0.900158107281 0.432866752148 -1001.452453613281;
-0.034808583558 0.434631049633 -0.899935662746 4471.351562500000;
0.000000000000 0.000000000000 0.000000000000 1.000000000000];

% TransSub2Master=TransSub2Master*WFOV_NFOVTrans;

TransSub3Master=[0.006639134604 -0.151031941175 0.988506615162 -1498.168579101563;
0.184727668762 0.971700012684 0.147223412991 -278.931213378906;
-0.982767343521 0.181627079844 0.034351024777 2089.546142578125;
0.000000000000 0.000000000000 0.000000000000 1.000000000000];

%% Writing Text File
fid=fopen('C:\Users\Abby.Eustace\Desktop\Kinect Azure\CalApril13Trans.txt','wt'); % Use for batch process
fprintf(fid,'###\nMaster.mkv');    
fprintf(fid,'\n%4.12f\t%4.12f\t%4.12f\t%4.12f',TransMaster');
fprintf(fid,'\n###\nSub1.mkv');    
fprintf(fid,'\n%4.12f\t%4.12f\t%4.12f\t%4.12f',TransSub1Master');
fprintf(fid,'\n###\nSub2.mkv');    
fprintf(fid,'\n%4.12f\t%4.12f\t%4.12f\t%4.12f',TransSub2Master');
fprintf(fid,'\n###\nSub3.mkv');    
fprintf(fid,'\n%4.12f\t%4.12f\t%4.12f\t%4.12f',TransSub3Master');
fclose(fid);
