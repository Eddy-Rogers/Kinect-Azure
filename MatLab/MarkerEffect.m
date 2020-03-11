clc; clear all ;close all

FilePath=uigetdir('C:\','Select the main folder where all .json files are');
Files=dir(fullfile(FilePath,'*.json'));
for f=1%1:length(Files)
fname = [FilePath, '\', Files(f).name];
end
val1 = jsondecode(fileread([FilePath, '\', Files(1).name]));
val2 = jsondecode(fileread([FilePath, '\', Files(2).name]));

NoMarkerPointFile='C:\Users\Abby.Eustace\Desktop\Kinect Azure\DataCollection\TinyFootMarkers\Test2\WO_Markers\AllTinyWOMarkers.20.ply';
MarkerPointFile='C:\Users\Abby.Eustace\Desktop\Kinect Azure\DataCollection\TinyFootMarkers\Test2\W_Markers\AllTinyWMarkers.20.ply';
MarkerPtCloud=pcread(MarkerPointFile);
NoMarkerPtCloud=pcread(NoMarkerPointFile);

figure
hold on
pcshow(MarkerPtCloud.Location,'b','VerticalAxis','Y','VerticalAxisDir','down')
pcshow(NoMarkerPtCloud.Location,'g','VerticalAxis','Y','VerticalAxisDir','down')
plot3(val1.frames(21).bodies.joint_positions(:,1),val1.frames(21).bodies.joint_positions(:,2),val1.frames(21).bodies.joint_positions(:,3),'gx')
plot3(val2.frames(21).bodies.joint_positions(:,1),val2.frames(21).bodies.joint_positions(:,2),val2.frames(21).bodies.joint_positions(:,3),'bo')


figure 
hold on
plot3(val1.frames(21).bodies.joint_positions(:,1),-val1.frames(21).bodies.joint_positions(:,2),val1.frames(21).bodies.joint_positions(:,3),'bx')
plot3(val2.frames(21).bodies.joint_positions(:,1),-val2.frames(21).bodies.joint_positions(:,2),val2.frames(21).bodies.joint_positions(:,3),'go')

%% Determining relative lengths of segments between markers and no markers
SkeletonMarkers=val1; %May need to change depending on fillename order
Skeleton=val2; %May need to change depending on filename order
countFrame=31;

JointsMarkers=SkeletonMarkers.frames(countFrame).bodies.joint_positions;
Joints=Skeleton.frames(countFrame).bodies.joint_positions;

LHip=[Joints(19,:)' JointsMarkers(19,:)'];
LKnee=[Joints(20,:)' JointsMarkers(20,:)'];
LAnkle=[Joints(21,:)' JointsMarkers(21,:)'];
LFoot=[Joints(22,:)' JointsMarkers(22,:)'];

RHip=[Joints(23,:)' JointsMarkers(23,:)'];
RKnee=[Joints(24,:)' JointsMarkers(24,:)'];
RAnkle=[Joints(25,:)' JointsMarkers(25,:)'];
RFoot=[Joints(26,:)' JointsMarkers(26,:)'];

ThighLengthL=vecnorm(LHip-LKnee);
ShankLengthL=vecnorm(LAnkle-LKnee);
FootLengthL=vecnorm(LFoot-LAnkle);
ThighLengthR=vecnorm(RHip-RKnee);
ShankLengthR=vecnorm(RAnkle-RKnee);
FootLengthR=vecnorm(RFoot-RAnkle);

ThighDiff=[diff(ThighLengthL) diff(ThighLengthR)];
ShankDiff=[diff(ShankLengthL) diff(ShankLengthR)];
FootDiff=[diff(FootLengthL) diff(ShankLengthR)];

T=table(ThighDiff',ShankDiff',FootDiff');
T.Properties.VariableNames={'Thigh Diff (mm)','Shank Diff (mm)','Foot Diff (mm)'};
T.Properties.RowNames={'Left','Right'};
T

% writetable(T,'C:\Users\Abby.Eustace\Desktop\Kinect Azure\ViconValidation\MarkerSetupValidation\SegmentLengthComparison.txt','Delimiter',' ') 

