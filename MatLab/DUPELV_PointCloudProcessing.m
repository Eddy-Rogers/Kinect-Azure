clc; clear all; close all;

plyPath='D:\DUPelv\S04\T04\';
jsonPath='D:\DUPelv\S04\jsonFiles\';
Files=dir([plyPath,'*.ply']);
FileNames=extractfield(Files,'name');
FileNames=string(FileNames);
% FileNames=getfield(Files,{1:end},'name');
jsonFile='Master_DUPELV04_04.json';
SkeletonData = jsondecode(fileread([jsonPath,jsonFile]));
PCDelay=32;%Delay to address the delay from the point cloud calibration 

for filei=1:length(Files)
    FileIndx=find(startsWith(FileNames,sprintf('Group%d_a',filei)));
    countFile=PCDelay+filei;
    PtCloud=pcread([plyPath Files(FileIndx).name]);
        x_coordinates = SkeletonData.frames(countFile).bodies.joint_positions(:,1);
        y_coordinates = SkeletonData.frames(countFile).bodies.joint_positions(:,2);
        z_coordinates = SkeletonData.frames(countFile).bodies.joint_positions(:,3);
        x_min = min(x_coordinates);
        x_max = max(x_coordinates);
        y_min = min(y_coordinates);
        y_max = max(y_coordinates);
        z_min = min(z_coordinates);
        z_max = min(z_coordinates);
        
         x_min=x_min-100;
        x_max=x_max+100;
        z_min=z_min-300;
        z_max=z_max+750;
        
        XFilterIndx=find(PtCloud.Location(:,1)<x_max & PtCloud.Location(:,1)>x_min);
        FilPtCloud=PtCloud.Location(XFilterIndx,:);
        YFilterIndx=find(FilPtCloud(:,2)<y_max & FilPtCloud(:,2)>y_min);
        FilPtCloud2=FilPtCloud(YFilterIndx,:);
        ZFilterIndx=find(FilPtCloud2(:,3)<z_max & FilPtCloud2(:,3)>z_min);
        FilPtCloudFinal=FilPtCloud2(ZFilterIndx,:);

        
figure
    hold on
    pcshow(FilPtCloudFinal,'VerticalAxis','Y','VerticalAxisDir','down')
    plot3(SkeletonData.frames(countFile).bodies.joint_positions(19:22,1),SkeletonData.frames(countFile).bodies.joint_positions(19:22,2),SkeletonData.frames(countFile).bodies.joint_positions(19:22,3),'rx','MarkerSize',10,'LineWidth',3);
   plot3(SkeletonData.frames(countFile).bodies.joint_positions(23:26,1),SkeletonData.frames(countFile).bodies.joint_positions(23:26,2),SkeletonData.frames(countFile).bodies.joint_positions(23:26,3),'gx','MarkerSize',10,'LineWidth',3);
%  plot3(SkeletonData.frames(countFile).bodies.joint_positions(:,1),SkeletonData.frames(countFile).bodies.joint_positions(:,2),SkeletonData.frames(countFile).bodies.joint_positions(:,3),'rx','MarkerSize',10,'LineWidth',3);
%  plot3(SkeletonData.frames(countFile).bodies.joint_positions(:,1),SkeletonData.frames(countFile).bodies.joint_positions(:,2),SkeletonData.frames(countFile).bodies.joint_positions(:,3),'gx','MarkerSize',10,'LineWidth',3);
%    
      xlim([-500 500])
    ylim([-800 1000])
    zlim([500 2000])
        view(74.21,-27.7031)
        camup([-0.069952470991766,-0.994770794597839,0.074415845203355])
%         axis equal
     Movie(filei)=getframe(gcf);
     close

end
videoname = ('D:\DUPelv\S04\T04.avi');

        v=VideoWriter(videoname);

        v.Quality=100;

        v.FrameRate=10;

        open(v)

        for i=1:length(Movie)

            writeVideo(v,Movie(i))

        end

        close(v)

    
