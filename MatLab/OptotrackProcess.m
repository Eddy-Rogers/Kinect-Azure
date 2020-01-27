%% Processing Optotrack data
clc;close all; clear all;
%% Import Probe Points to define bodies axis
ProbePointsPath=('C:\Users\Abby.Eustace\Desktop\Kinect Azure\Validation\HDL_Test_10_21_19_2\probe data');
FemurFile='femur_med_lat_troch.txt';
TibiaFile='tibia_latknee_med_latank_med.txt';
FootFile='foot_lat_med_toetip.txt';

% Knee Med, Knee Lat, TRO
FemurProbePts=dlmread([ProbePointsPath,'\',FemurFile],'');
% Knee Lat, Knee Med, Ankle Lat, Ankle Med
TibiaProbePts=dlmread([ProbePointsPath,'\',TibiaFile],'');
% Ankle Lat, Ankle Med, ToeTip
FootProbePts=dlmread([ProbePointsPath,'\',FootFile],'');
%**************************************************************************
%Creating Coordinate System of Femur, centered at the distal end of the
%femur
Femur_O=0.5.*(FemurProbePts(2,2:4)+FemurProbePts(1,2:4));
femur_z_vec=
%**************************************************************************
% Creating Coordinate System of Tibia, centered at the proximal end of the tibia 
Tibia_O=0.5.*(TibiaProbePts(1,2:4)+TibiaProbePts(2,2:4));%Tibia Origin
tibia_z_vec=(Tibia_O-(0.5.*(TibiaProbePts(3,2:4)+TibiaProbePts(4,2:4))))/norm(Tibia_O-(0.5.*(TibiaProbePts(3,2:4)+TibiaProbePts(4,2:4))));
tibia_temp=(TibiaProbePts(1,2:4)-TibiaProbePts(2,2:4))/norm(TibiaProbePts(1,2:4)-TibiaProbePts(2,2:4));
tibia_y_vec=cross(tibia_z_vec,tibia_temp);
tibia_x_vec=cross(tibia_y_vec,tibia_z_vec);
% % % % tibia_z_vec=(TibiaProbePts(1,2:4)-TibiaProbePts(2,2:4))/norm(TibiaProbePts(1,2:4)-TibiaProbePts(2,2:4));
% % % % tibia_z_vec=(tibia_z_vec-Tibia_mid)/norm(tibia_z_vec-Tibia_mid);
% % % % 
% % % % Tibia_ankle_mid=mean(TibiaProbePts(3:4,2:4),1);
% % % % tibia_y_vec=(Tibia_mid-Tibia_ankle_mid)/norm(Tibia_mid-Tibia_ankle_mid);
% % % % tibia_y_vec=(tibia_y_vec-Tibia_mid)/norm(tibia_y_vec-Tibia_mid);
% % % % tibia_x_vec=cross(tibia_y_vec,tibia_z_vec)/norm(cross(tibia_y_vec,tibia_z_vec));
% % % % tibia_y_vec=cross(tibia_z_vec,tibia_x_vec)/norm(cross(tibia_z_vec,tibia_x_vec));
%**************************************************************************
% % % %Creating Coordinate System of Foot, center at the proximal end of the foot
% % % Foot_mid=mean(FootProbePts(1:2,2:4),1);
% % % foot_z_vec=(FootProbePts(1,2:4)-FootProbePts(2,2:4))/norm(FootProbePts(1,2:4)-FootProbePts(2,2:4));
% % % foot_z_vec=(foot_z_vec-Foot_mid)/norm(foot_z_vec-Foot_mid);
% % % foot_x_vec=(FootProbePts(3,2:4)-Foot_mid)/norm(FootProbePts(3,2:4)-Foot_mid);
% % % foot_y_vec=cross(foot_z_vec,foot_x_vec)/norm(cross(foot_z_vec,foot_x_vec));
% % % foot_x_vec=cross(foot_y_vec,foot_z_vec)/norm(cross(foot_y_vec,foot_z_vec));

  
% % % % figure
% % % % hold on
% % % % plot3([Tibia_O(1,1);185.*tibia_x_vec(1,1)],[Tibia_O(1,2);185.*tibia_x_vec(1,2)],[Tibia_O(1,3);185.*tibia_x_vec(1,3)],'r');
% % % % plot3([Tibia_O(1,1);185.*tibia_y_vec(1,1)],[Tibia_O(1,2);185.*tibia_y_vec(1,2)],[Tibia_O(1,3);185.*tibia_y_vec(1,3)],'g');
% % % % plot3([Tibia_O(1,1);185.*tibia_z_vec(1,1)],[Tibia_O(1,2);185.*tibia_z_vec(1,2)],[Tibia_O(1,3);185.*tibia_z_vec(1,3)],'b');
% % % % plot3(TibiaProbePts(1,2),TibiaProbePts(1,3),TibiaProbePts(1,4),'gx')
% % % % plot3(TibiaProbePts(2,2),TibiaProbePts(2,3),TibiaProbePts(2,4),'kx')
% % % % plot3(TibiaProbePts(3,2),TibiaProbePts(3,3),TibiaProbePts(3,4),'rx')
% % % % plot3(TibiaProbePts(4,2),TibiaProbePts(4,3),TibiaProbePts(4,4),'bx')
% % % % 
% % % % figure
% % % % hold on
% % % % plot3([Foot_mid(1,1);185*foot_x_vec(1,1)],[Foot_mid(1,2);185*foot_x_vec(1,2)],[Foot_mid(1,3);185*foot_x_vec(1,3)],'r');
% % % % plot3([Foot_mid(1,1);185*foot_y_vec(1,1)],[Foot_mid(1,2);185*foot_y_vec(1,2)],[Foot_mid(1,3);185*foot_y_vec(1,3)],'g');
% % % % plot3([Foot_mid(1,1);185*foot_z_vec(1,1)],[Foot_mid(1,2);185*foot_z_vec(1,2)],[Foot_mid(1,3);185*foot_z_vec(1,3)],'b');
% % % % plot3(FootProbePts(1,2),FootProbePts(1,3),FootProbePts(1,4),'gx')
% % % % plot3(FootProbePts(2,2),FootProbePts(2,3),FootProbePts(2,4),'kx')
% % % % plot3(FootProbePts(3,2),FootProbePts(3,3),FootProbePts(3,4),'rx')

%% Load Optotrack Data
OptotrackPath=('C:\Users\Abby.Eustace\Desktop\Kinect Azure\Validation\HDL_Test_10_21_19_2\');
TrialFile=('HDL_Test_10_21_19_2_001_6d.csv');

OptotrackData=csvread([OptotrackPath, TrialFile],5,14);

ThighOptotrackData=zeros(3,3,size(OptotrackData,1));
ShankOptotrackData=zeros(3,3,size(OptotrackData,1));
FootOptotrackData=zeros(3,3,size(OptotrackData,1));


ThighIndx=1; 
ShankIndx=ThighIndx+13;
FootIndx=ShankIndx+13;
for x=1:4
    for frame=1:size(OptotrackData,1)
        
        ThighOptotrackData(x,2:4,frame)=OptotrackData(frame,ThighIndx:ThighIndx+2);
        ShankOptotrackData(x,2:4,frame)=OptotrackData(frame,ShankIndx:ShankIndx+2);
        FootOptotrackData(x,2:4,frame)=OptotrackData(frame,FootIndx:FootIndx+2);
        
    end
    ThighIndx=ThighIndx+3;
    ShankIndx=ShankIndx+3;
    FootIndx=FootIndx+3;
end
