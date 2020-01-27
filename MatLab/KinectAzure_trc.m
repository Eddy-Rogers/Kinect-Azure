%% Create the trc file for OpenSim from Kinect Azure
clc; clear all ;close all

fname = 'C:\Users\Abby.Eustace\Desktop\Kinect Azure\Cob Kinect Azure\x64\Debug\BrittanyJoints.json';
SkeletonData = jsondecode(fileread(fname));
JointLabels=SkeletonData.joint_names;
Time=extractfield(SkeletonData.frames,'timestamp_usec');
Time=Time.*10^-6;
Frames=SkeletonData.frames;
FrameID=extractfield(SkeletonData.frames,'frame_id');
JointData=zeros(length(Frames),32*3);

% Determing the last frame where a body was tracked
Bodies=extractfield(SkeletonData.frames,'num_bodies');
LastBodyDetected=find(Bodies==0,1,'first');

% Creating a matrix of joint data where columns are x,y,z cord of joints
% and rows are the frames
for ii=1:LastBodyDetected-1
    count=1;
    for jj=1:32 %Num of Joints
        JointData(ii,count:count+2)=SkeletonData.frames(ii).bodies.joint_positions(jj,:);
        count=count+3;
    end
end

%% Transforming Kinect Data orientation to OpenSim Orientation

JointsOpenSimOrien=zeros(length(Frames),32*3);
for p=1:3:size(JointData,2)
    JointsOpenSimOrien(:,p)=-JointData(:,p+2);
    JointsOpenSimOrien(:,p+1)=-JointData(:,p+1);
    JointsOpenSimOrien(:,p+2)=-JointData(:,p);
end
OpenSimData=zeros(length(Frames),32*3+2);
OpenSimData(:,1)=FrameID';
OpenSimData(:,2)=Time';
OpenSimData(:,3:end)=JointsOpenSimOrien;


% % % 
% % % %% Filter Raw Marker Data
% % % OpenSimDataPrimeFiltered=OpenSimDataPrime;
% % % fc=3/15;% Cutoff Frequency 3 Hz
% % % [b,a] = butter(4,fc); % Using a forth order butterworth filter 
% % % for j=3:size(OpenSimDataPrime,2)
% % % % Useing filtfilt a zero delay filter with the forth order butterworth 
% % % OpenSimDataPrimeFiltered(:,j) = filtfilt(b,a,OpenSimDataPrime(:,j));
% % % 
% % % end
% % % 

OpenSimData=OpenSimData(1:LastBodyDetected-1,:)';
%% Create a trc file 
OpenSimLabel={'Frame#','Time',JointLabels{:}};

Row1={'PathFileType','4','(X/Y/Z)','KinectAzureSkeleton.trc'};
Row2={'DateRate','CameraRate','NumFrames','NumMarkers','Units','OrigDateRate','OrigDataStartFrame','OrigNumFrames'};
Row3={'30','30',num2str(LastBodyDetected-1),'32','mm','30','1',num2str(LastBodyDetected-1)};
Row4=OpenSimLabel;%{'Frame#','Time','SPNB','T10','CLAV','NECK','HEAD','LSHO','LELW','LWST','LHAD','LHIP','LKNE','LANK','LTIP','LHTIP','LTHB','RSHO','RELW','RWST','RHAD','RHIP','RKNE','RANK','RTIP','RHTIP','RTHB'};

cord=1;
for r=1:32 %Num of Joint locations
Row5(1,cord:cord+2)={sprintf('X%d',r), sprintf('Y%d',r),sprintf('Z%d',r)};%,'X2',	'Y2','Z2','X3','Y3','Z3','X4','Y4','Z4','X5','Y5','Z5',	'X6','Y6','Z6','X7','Y7','Z7','X8','Y8','Z8','X9','Y9','Z9','X10','Y10','Z10','X11','Y11','Z11','X12','Y12','Z12','X13','Y13','Z13','X14','Y14','Z14','X15','Y15','Z15','X16','Y16','Z16','X17','Y17','Z17','X18','Y18','Z18','X19','Y19','Z19','X20','Y20','Z20','X21','Y21','Z21','X22','Y22','Z22','X23','Y23','Z23','X24','Y24','Z24','X25','Y25','Z25'};
cord=cord+3;
end

fid=fopen(['C:\Users\Abby.Eustace\Desktop\Kinect Azure\OpenSim\BrittanyWalk.trc'],'wt'); % Use for batch process
fprintf(fid,'%s\t%s\t%s\t%s',Row1{:});
fprintf(fid,'\n%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s',Row2{:});
fprintf(fid,'\n%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s',Row3{:});    
fprintf(fid,'\n%s\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s\t\t\t%s',Row4{:});
fprintf(fid,'\n\t\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n',Row5{:});
    
 fprintf(fid,'\n%d\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f\t%7.6f',...
        OpenSimData);
fclose(fid);