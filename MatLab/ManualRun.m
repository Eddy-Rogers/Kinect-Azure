 clc; clear all; 
 
mkvPath = ('C:\Users\Abby.Eustace\Desktop\Kinect Azure\Azure Kinect SDK v1.3.0\tools\'); 
Path = ('C:\Users\Abby.Eustace\Desktop\Kinect Azure\Azure Kinect SDK v1.3.0\tools\Variability\');
exePath=('C:\Users\Abby.Eustace\Desktop\Kinect Azure\Cob Kinect Azure\x64\Debug\');
File=input('What is the filename');

collectingEXE='k4arecorder.exe';
exe='OnlyJointProcessing.exe';
      jsonFile=[File '.json'];
       mkvFile=[File '.mkv'];
       %% Collect Kinect Data
        system(['"',mkvPath,collectingEXE,'" -l 5 "',Path,mkvFile,'"']);
         
%% Process the mkv to get the Key Points
        system(['"',exePath,exe,'" "',Path,mkvFile,'" "',Path,jsonFile,'"']);

        pause(2)
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

        frames=SkeletonData.frames;
        
        
        % Determing the last frame where a body was tracked
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



        for ii=1:length(frames)

            % Calculate the Right knee angle as the angle between the thigh and
            % shank vectors
            RightHipJointLocation=KinectDataFiltered(RightHipIndx,:,ii);
            RightKneeJointLocation=KinectDataFiltered(RightKneeIndx,:,ii);
            RightAnkleJointLocation=KinectDataFiltered(RightAnkleIndx,:,ii);

            RightThigh=RightHipJointLocation-RightKneeJointLocation;
            RightShank=RightAnkleJointLocation-RightKneeJointLocation;

            RightKneeAngle(ii)=acos(dot(RightThigh,RightShank)/(norm(RightThigh)*norm(RightShank)));

            %Calculate the Left knee angle as the angle between the thigh and shank
            %vectors
             LeftHipJointLocation=KinectDataFiltered(LeftHipIndx,:,ii);
            LeftKneeJointLocation=KinectDataFiltered(LeftKneeIndx,:,ii);
            LeftAnkleJointLocation=KinectDataFiltered(LeftAnkleIndx,:,ii);

            LeftThigh=LeftHipJointLocation-LeftKneeJointLocation;
            LeftShank=LeftAnkleJointLocation-LeftKneeJointLocation;

            LeftKneeAngle(ii)=acos(dot(LeftThigh,LeftShank)/(norm(LeftThigh)*norm(LeftShank)));

        end

        RightKneeAngle=RightKneeAngle.*(180/pi); %Convert angle from radians to degrees
        RightKneeAngle=180-RightKneeAngle; %Match the knee angle to the method measured in OpenSim

        LeftKneeAngle=LeftKneeAngle.*(180/pi); %Convert angle from radians to degrees
        LeftKneeAngle=180-LeftKneeAngle; %Match the knee angle to the method measured in OpenSim
        
        figure
        plot(LeftKneeAngle)
        title(sprintf('Left Knee Angle %s',File))
        ylabel('Knee Angle (deg)')
        
        
        figure
        plot(RightKneeAngle)
        title(sprintf('Right Knee Angle %s',File))
        ylabel('Knee Angle (deg)')
