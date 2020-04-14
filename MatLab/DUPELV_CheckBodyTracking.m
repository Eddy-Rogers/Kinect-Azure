clc; clear all; close all;

Path = ('D:\DUPelv\S05\mvkFiles\');
File={'01','02','03','04','05','06','07','08','09','10','11','12'};

FileName='Master_DUPELV05_';
for filei=12%1:length(File)
jsonFile=[FileName File{filei} '.json'];

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
        
        figure 
        plot(RightKneeAngle)
        title(sprintf('Right Knee Angle %d',filei))
%         axis equal
        figure
        plot(LeftKneeAngle)
%         axis equal
        title(sprintf('Left Knee Angle %d',filei))
        
end