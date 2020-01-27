clc; clear all ;close all

FilePath=uigetdir('C:\','Select the main folder where all .json files are');
Files=dir(fullfile(FilePath,'*.json'));
for f=1%1:length(Files)
fname = [FilePath, '\', Files(f).name];
end
val1 = jsondecode(fileread([FilePath, '\', Files(1).name]));
val2 = jsondecode(fileread([FilePath, '\', Files(2).name]));

MarkerPointFile='C:\Users\Abby.Eustace\Desktop\Kinect Azure\DataCollection\MarkersEffect\t_pose_w_markers.1.ply';
NoMarkerPointFile='C:\Users\Abby.Eustace\Desktop\Kinect Azure\DataCollection\MarkersEffect\t_pose_wo_markers.1.ply';


figure
hold on
plot3(val1.frames(1).bodies.joint_positions(:,1),val1.frames(1).bodies.joint_positions(:,2),val1.frames(1).bodies.joint_positions(:,3),'bx')
plot3(val2.frames(1).bodies.joint_positions(:,1),val2.frames(1).bodies.joint_positions(:,2),val2.frames(1).bodies.joint_positions(:,3),'go')
