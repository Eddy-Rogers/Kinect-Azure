%% Kinect Variability 
clc; clear all; close all

count=0;
for filei=49:70
    count = count + 1;
Path = ('D:\Variability\');
File=sprintf('T%d',filei);
jsonFile=[File '.json'];

%% Determine the knee angle from the key points        
        SkeletonData = jsondecode(fileread([Path,jsonFile]));
        JointLabels=SkeletonData.joint_names;
        RightKneeIndx=find(strcmp(JointLabels,'KNEE_RIGHT'));
        RightHipIndx=find(strcmp(JointLabels,'HIP_RIGHT'));
        RightAnkleIndx=find(strcmp(JointLabels,'ANKLE_RIGHT'));
        RightToeIndx=find(strcmp(JointLabels,'FOOT_RIGHT'));
        LeftKneeIndx=find(strcmp(JointLabels,'KNEE_LEFT'));
        LeftHipIndx=find(strcmp(JointLabels,'HIP_LEFT'));
        LeftAnkleIndx=find(strcmp(JointLabels,'ANKLE_LEFT'));
         LeftToeIndx=find(strcmp(JointLabels,'FOOT_LEFT'));
        PelvisIndx=find(strcmp(JointLabels,'PELVIS'));

        frames=[];
        frames=SkeletonData.frames;
        
        
        % Determing the last frame where a body was tracked
        Bodies=[];
        Bodies=extractfield(SkeletonData.frames,'num_bodies');
        
        InitialIndx=find(Bodies==1 | Bodies==2 ,1,'First');
        FinalTempIndx=find(flip(Bodies)==1 | flip(Bodies)==2 ,1,'First');
        FinalIndx=length(Bodies)-FinalTempIndx+1;
        
        frames=frames(InitialIndx:FinalIndx);
        Time=extractfield(frames,'timestamp_usec');
        Time=Time.*10^-6;
        FrameID=extractfield(frames,'frame_id');
        JointData=zeros(length(frames),32*3);
        
        
        % % % if isempty(find(Bodies==0,1,'first'))==1
        % % %     LastBodyDetected=length(Bodies);
        % % % else
        % % %     LastBodyDetected=find(Bodies==0,1,'first');
        % % %     LastBodyDetected=LastBodyDetected-1;
        % % % end
        % % % KinectData=zeros(32,3,LastBodyDetected);
        % % % for ff=1:LastBodyDetected
        % % %     KinectData(:,:,ff)=val.frames(ff).bodies.joint_positions;
        % % % end
        
        KinectData=zeros(32,3,length(frames));
        for ff=1:length(frames)
            if isempty(frames(ff).bodies)==1
                
            elseif frames(ff).num_bodies==2
               
            else
                 KinectData(:,:,ff)=frames(ff).bodies.joint_positions;
            end
            
        end
        KinectData(KinectData==0)=nan;
        KinectDataInterp=zeros(32,3,length(frames));
        sf=1;
        for j=1:32
            for jj=1:3
                KinectDataInterp(j,jj,:)=csaps(1:length(frames),KinectData(j,jj,:),sf,1:length(frames));
            end
        end
        
        fc=3/15;% Cutoff Frequency 3 Hz
        [b,a] = butter(4,fc); % Using a forth order butterworth filter
        
        % Using filtfilt a zero delay filter with the forth order butterworth
        KinectDataFiltered=zeros(32,3,length(frames));
        for j=1:32
            for jj=1:3
                KinectDataFiltered(j,jj,:)=filtfilt(b,a,squeeze(KinectDataInterp(j,jj,:)));
            end
        end
        
           RightKneeAngle=zeros(length(frames),1);
        LeftKneeAngle=zeros(length(frames),1);



        for rl=1:length(frames)

            % Calculate the Right knee angle as the angle between the thigh and
            % shank vectors
            RightHipJointLocation=KinectDataFiltered(RightHipIndx,:,rl);
            RightKneeJointLocation=KinectDataFiltered(RightKneeIndx,:,rl);
            RightAnkleJointLocation=KinectDataFiltered(RightAnkleIndx,:,rl);

            RightThigh=RightHipJointLocation-RightKneeJointLocation;
            RightShank=RightAnkleJointLocation-RightKneeJointLocation;

            RightKneeAngle(rl)=acos(dot(RightThigh,RightShank)/(norm(RightThigh)*norm(RightShank)));

            %Calculate the Left knee angle as the angle between the thigh and shank
            %vectors
             LeftHipJointLocation=KinectDataFiltered(LeftHipIndx,:,rl);
            LeftKneeJointLocation=KinectDataFiltered(LeftKneeIndx,:,rl);
            LeftAnkleJointLocation=KinectDataFiltered(LeftAnkleIndx,:,rl);

            LeftThigh=LeftHipJointLocation-LeftKneeJointLocation;
            LeftShank=LeftAnkleJointLocation-LeftKneeJointLocation;

            LeftKneeAngle(rl)=acos(dot(LeftThigh,LeftShank)/(norm(LeftThigh)*norm(LeftShank)));

        end

        RightKneeAngle=RightKneeAngle.*(180/pi); %Convert angle from radians to degrees
        RightKneeAngle=180-RightKneeAngle; %Match the knee angle to the method measured in OpenSim

        LeftKneeAngle=LeftKneeAngle.*(180/pi); %Convert angle from radians to degrees
        LeftKneeAngle=180-LeftKneeAngle; %Match the knee angle to the method measured in OpenSim
        
      


%% Determining Best Method for Cutting to Gait Cycle
HeelMarkerR=squeeze(KinectDataFiltered(RightAnkleIndx,:,:));
ToeMarkerR=squeeze(KinectDataFiltered(RightToeIndx,:,:));
PelvisMarker=squeeze(KinectDataFiltered(PelvisIndx,:,:));
HeelMarkerL=squeeze(KinectDataFiltered(LeftAnkleIndx,:,:));
ToeMarkerL=squeeze(KinectDataFiltered(LeftToeIndx,:,:));

%% Calculating Temp\Spat Measures
% DiffPelvisHeelVert=zeros(1,length(HeelMarker));
DiffPelvisHeelR=zeros(1,length(HeelMarkerR));
DiffPelvisToeR=zeros(1,length(HeelMarkerR));
DiffPelvisHeelL=zeros(1,length(HeelMarkerL));
DiffPelvisToeL=zeros(1,length(HeelMarkerL));
for rl=1:length(HeelMarkerR)
DiffPelvisHeelR(rl)=HeelMarkerR(3,rl)-PelvisMarker(3,rl);
DiffPelvisToeR(rl)=ToeMarkerR(3,rl)-PelvisMarker(3,rl);
DiffPelvisHeelL(rl)=HeelMarkerL(3,rl)-PelvisMarker(3,rl);
DiffPelvisToeL(rl)=ToeMarkerL(3,rl)-PelvisMarker(3,rl);
end

% % % Start of mannually defining gait events
% % figure
% % hold on
% % yyaxis left
% %  plot(DiffPelvisToe,'g')
% % plot(DiffPelvisHeel,'b')
% % yyaxis right
% % plot(RightKneeAngle,'r')
% % drawnow

MinPHIndxR=islocalmin(DiffPelvisHeelR,'MinProminence',100);
MinPHIndxL=islocalmin(DiffPelvisHeelL,'MinProminence',100);

PHInitIndxR=find(MinPHIndxR==1,2,'First');
PHInitIndxL=find(MinPHIndxL==1,2,'First');

        
% %         figure
% %         plot(RightKneeAngle)
% % %       xline(AccelInitIndx,'r')
% %         xline(PHInitIndxR(1),'g')
% %         xline(PHInitIndxR(2),'g')
% % %       xline(PHInitIndx(3),'g')
% %         title(sprintf('Right Knee Angle %s',File))
% %         ylabel('Knee Angle (deg)')


RightKneeCut{:,count}=RightKneeAngle(PHInitIndxR(1):PHInitIndxR(2));
LeftKneeCut{:,count}=LeftKneeAngle(PHInitIndxL(1):PHInitIndxL(2));

 RightKneeNorm(:,count)=spline(linspace(1,length(RightKneeAngle(PHInitIndxR(1):PHInitIndxR(2))),length(RightKneeAngle(PHInitIndxR(1):PHInitIndxR(2)))),RightKneeAngle(PHInitIndxR(1):PHInitIndxR(2)),...
                linspace(1,length(RightKneeAngle(PHInitIndxR(1):PHInitIndxR(2))),101));
           
            LeftKneeNorm(:,count)=spline(linspace(1,length(LeftKneeAngle(PHInitIndxL(1):PHInitIndxL(2))),length(LeftKneeAngle(PHInitIndxL(1):PHInitIndxL(2)))),LeftKneeAngle(PHInitIndxL(1):PHInitIndxL(2)),...
                linspace(1,length(LeftKneeAngle(PHInitIndxL(1):PHInitIndxL(2))),101));

% %   figure
% %   hold on
% %         plot(LeftKneeAngle)
% % 
% %         title(sprintf('Left Knee Angle %s',File))
% %         ylabel('Knee Angle (deg)')
        

        
end

figure 
hold on
plot(RightKneeNorm(:,:))
title('Right Knee Angle')

figure
hold on
plot(LeftKneeNorm(:,:))
title('Left Knee Angle')