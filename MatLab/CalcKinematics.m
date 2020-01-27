clc; clear all ;%close all

FilePath=uigetdir('C:\','Select the main folder where all .json files are');
Files=dir(fullfile(FilePath,'*.json'));
for f=3%1:length(Files)
fname = [FilePath, '\', Files(f).name];
val = jsondecode(fileread(fname));


frames=val.frames;
JointLabels=val.joint_names;
Time=extractfield(val.frames,'timestamp_usec');
Bodies=extractfield(val.frames,'num_bodies');
Frequency=1./(diff(Time).*10^-6);
RightKneeIndx=find(strcmp(JointLabels,'KNEE_RIGHT'));
RightHipIndx=find(strcmp(JointLabels,'HIP_RIGHT'));
RightAnkleIndx=find(strcmp(JointLabels,'ANKLE_RIGHT'));
LeftKneeIndx=find(strcmp(JointLabels,'KNEE_LEFT'));
LeftHipIndx=find(strcmp(JointLabels,'HIP_LEFT'));
LeftAnkleIndx=find(strcmp(JointLabels,'ANKLE_LEFT'));
if isempty(find(Bodies==0,1,'first'))==1
    LastBodyDetected=length(Bodies);
else
    LastBodyDetected=find(Bodies==0,1,'first');
    LastBodyDetected=LastBodyDetected-1;
end

RightKneeAngle=zeros(135,1);%LastBodyDetected,1);
LeftKneeAngle=zeros(135,1);%LastBodyDetected,1);

for ii=25:160%1:LastBodyDetected
    % Calculate the Right knee angle as the angle between the thigh and
    % shank vectors
    RightHipJointLocation=val.frames(ii).bodies.joint_positions(RightHipIndx,:);
    RightKneeJointLocation=val.frames(ii).bodies.joint_positions(RightKneeIndx,:);
    RightAnkleJointLocation=val.frames(ii).bodies.joint_positions(RightAnkleIndx,:);
    
    RightThigh=RightHipJointLocation-RightKneeJointLocation;
    RightShank=RightAnkleJointLocation-RightKneeJointLocation;
    
    RightKneeAngle(ii)=acos(dot(RightThigh,RightShank)/(norm(RightThigh)*norm(RightShank)));
    
    %Calculate the Left knee angle as the angle between the thigh and shank
    %vectors
     LeftHipJointLocation=val.frames(ii).bodies.joint_positions(LeftHipIndx,:);
    LeftKneeJointLocation=val.frames(ii).bodies.joint_positions(LeftKneeIndx,:);
    LeftAnkleJointLocation=val.frames(ii).bodies.joint_positions(LeftAnkleIndx,:);
    
    LeftThigh=LeftHipJointLocation-LeftKneeJointLocation;
    LeftShank=LeftAnkleJointLocation-LeftKneeJointLocation;
    
    LeftKneeAngle(ii)=acos(dot(LeftThigh,LeftShank)/(norm(LeftThigh)*norm(LeftShank)));
   
end
 
RightKneeAngle=RightKneeAngle.*(180/pi); %Convert angle from radians to degrees
RightKneeAngle=RightKneeAngle-180; %Match the knee angle to the method measured in OpenSim

LeftKneeAngle=LeftKneeAngle.*(180/pi); %Convert angle from radians to degrees
LeftKneeAngle=LeftKneeAngle-180; %Match the knee angle to the method measured in OpenSim

figure
hold on
plot(-RightKneeAngle,'g')
plot(-LeftKneeAngle,'r')
hold off
% % figure  
% % plot(-val.frames(ii).bodies.joint_positions(1:27,1),-val.frames(ii).bodies.joint_positions(:,2),'x')   
end