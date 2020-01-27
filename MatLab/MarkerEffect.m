clc; clear all ;close all

FilePath=uigetdir('C:\','Select the main folder where all .json files are');
Files=dir(fullfile(FilePath,'*.json'));
for f=1%1:length(Files)
fname = [FilePath, '\', Files(f).name];
end
val1 = jsondecode(fileread([FilePath, '\', Files(1).name]));
val2 = jsondecode(fileread([FilePath, '\', Files(2).name]));

NoMarkerPointFile='C:\Users\Abby.Eustace\Desktop\Kinect Azure\DataCollection\MarkersEffect\SmallFullLowerMarkers\WO_Markers\AposeWOFullLowerSmallMarkers.20.ply';
MarkerPointFile='C:\Users\Abby.Eustace\Desktop\Kinect Azure\DataCollection\MarkersEffect\SmallFullLowerMarkers\W_Markers\AposeWFullLowerSmallMarkers.20.ply';
MarkerPtCloud=pcread(MarkerPointFile);
NoMarkerPtCloud=pcread(NoMarkerPointFile);

figure
hold on
pcshow(MarkerPtCloud.Location,'b','VerticalAxis','Y','VerticalAxisDir','down')
pcshow(NoMarkerPtCloud.Location,'g','VerticalAxis','Y','VerticalAxisDir','down')
plot3(val1.frames(21).bodies.joint_positions(:,1),val1.frames(21).bodies.joint_positions(:,2),val1.frames(21).bodies.joint_positions(:,3),'bx')
plot3(val2.frames(21).bodies.joint_positions(:,1),val2.frames(21).bodies.joint_positions(:,2),val2.frames(21).bodies.joint_positions(:,3),'go')


figure 
hold on
plot3(val1.frames(21).bodies.joint_positions(:,1),-val1.frames(21).bodies.joint_positions(:,2),val1.frames(21).bodies.joint_positions(:,3),'bx')
plot3(val2.frames(21).bodies.joint_positions(:,1),-val2.frames(21).bodies.joint_positions(:,2),val2.frames(21).bodies.joint_positions(:,3),'go')
