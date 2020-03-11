clc; clear all ;close all
MasterPointFilePath=dir('R:\Research Common\HDL\Personal Folders\Eustace\KinectViconValidation\SetupTestData\Test2(Feb17)\PlyFiles\Master\*.ply');
Sub1PointFilePath=dir('R:\Research Common\HDL\Personal Folders\Eustace\KinectViconValidation\SetupTestData\Test2(Feb17)\PlyFiles\Sub1\*.ply');
Sub2PointFilePath=dir('R:\Research Common\HDL\Personal Folders\Eustace\KinectViconValidation\SetupTestData\Test2(Feb17)\PlyFiles\Sub2\*.ply');
Sub3PointFilePath=dir('R:\Research Common\HDL\Personal Folders\Eustace\KinectViconValidation\SetupTestData\Test2(Feb17)\PlyFiles\Sub3\*.ply');

MasterFileLength=length(MasterPointFilePath);
Sub1FileLength=length(Sub1PointFilePath);
Sub2FileLength=length(Sub2PointFilePath);
Sub3FileLength=length(Sub3PointFilePath);

LeastFiles=min([MasterFileLength,Sub1FileLength,Sub2FileLength,Sub3FileLength]);
angle=1.3*(pi/180);
WFOV_NFOVTrans=[1 0 0 0;
    0 cos(angle) -sin(angle)  0;
    0 sin(angle) cos(angle)  0;
    0 0 0 1];

TransSub1Master=[0.003262548940 0.158023983240 -0.987429857254 2240.754882812500
-0.134737223387 0.978500425816 0.156149774790 -326.543151855469
0.990876019001 0.132534116507 0.024484118447 2489.085449218750
0.000000000000 0.000000000000 0.000000000000 1.000000000000];

TransSub2Master=[ -0.999287605286 0.018243510276 -0.033037651330 117.151992797852
0.009763699956 0.970568001270 0.240629211068 -600.226440429688
0.036455206573 0.240135207772 -0.970054686069 5110.304199218750
0.000000000000 0.000000000000 0.000000000000 1.000000000000];

TransSub3Master=[0.007361589931 -0.138939142227 0.990273535252 -2110.854736328125
0.119770132005 0.983294010162 0.137069523335 -259.883605957031
-0.992774367332 0.117596141994 0.023879365996 2609.124267578125
0.000000000000 0.000000000000 0.000000000000 1.000000000000];

k=1;

for countFile=1:5%1:LeastFiles
    
if exist([MasterPointFilePath(1).folder,'\',sprintf('Sub00Feb17MasterTrial02.%d.ply',countFile)])==2 && ...  
exist([Sub1PointFilePath(1).folder,'\',sprintf('Sub00Feb17Sub1Trial02.%d.ply',countFile)])==2 && ...  
exist([Sub2PointFilePath(1).folder,'\',sprintf('Sub00Feb17Sub2Trial02.%d.ply',countFile)])==2 && ...  
exist([Sub3PointFilePath(1).folder,'\',sprintf('Sub00Feb17Sub3Trial02.%d.ply',countFile)])==2
    
MasterPointFile=[MasterPointFilePath(1).folder,'\',sprintf('Sub00Feb17MasterTrial02.%d.ply',countFile)];  
Sub1PointFile=[Sub1PointFilePath(1).folder,'\',sprintf('Sub00Feb17Sub1Trial02.%d.ply',countFile)];  
Sub2PointFile=[Sub2PointFilePath(1).folder,'\',sprintf('Sub00Feb17Sub2Trial02.%d.ply',countFile)];  
Sub3PointFile=[Sub3PointFilePath(1).folder,'\',sprintf('Sub00Feb17Sub3Trial02.%d.ply',countFile)];  


MasterPtCloud=pcread(MasterPointFile);
    Sub1PtCloud=pcread(Sub1PointFile);
      Sub2PtCloud=pcread(Sub2PointFile);
        Sub3PtCloud=pcread(Sub3PointFile);
    
    LengthMaster=length(MasterPtCloud.Location);
    LengthSub1=length(Sub1PtCloud.Location);
     LengthSub2=length(Sub2PtCloud.Location);
      LengthSub3=length(Sub3PtCloud.Location);
      
         MasterPtCloudTemp=ones(LengthMaster,4);
    MasterPtCloudTemp(:,1:3)=MasterPtCloud.Location(:,:);
    
    Sub1PtCloudTemp=ones(LengthSub1,4);
    Sub1PtCloudTemp(:,1:3)=Sub1PtCloud.Location(:,:);
 
     Sub2PtCloudTemp=ones(LengthSub2,4);
    Sub2PtCloudTemp(:,1:3)=Sub2PtCloud.Location(:,:);
    
     Sub3PtCloudTemp=ones(LengthSub3,4);
    Sub3PtCloudTemp(:,1:3)=Sub3PtCloud.Location(:,:);
    
    for m=1:LengthMaster
       MasterPtCloudTrans(m,:)=WFOV_NFOVTrans*MasterPtCloudTemp(m,:)';
   end
    
   for s1=1:LengthSub1
       Sub1PtCloudTrans(s1,:)=TransSub1Master*Sub1PtCloudTemp(s1,:)';
   end
    
   for s2=1:LengthSub2
       Sub2PtCloudTrans(s2,:)=TransSub2Master*WFOV_NFOVTrans*Sub2PtCloudTemp(s2,:)';
   end
   
   for s3=1:LengthSub3
       Sub3PtCloudTrans(s3,:)=TransSub3Master*Sub3PtCloudTemp(s3,:)';
   end
      MasterPtCloudFinal=pointCloud(MasterPtCloudTrans(:,1:3));
   Sub1PtCloudFinal=pointCloud(Sub1PtCloudTrans(:,1:3));
    Sub2PtCloudFinal=pointCloud(Sub2PtCloudTrans(:,1:3));
     Sub3PtCloudFinal=pointCloud(Sub3PtCloudTrans(:,1:3));
     
     FinalPointCloudtemp1=pcmerge(MasterPtCloudFinal,Sub1PtCloudFinal,0.1);
    FinalPointCloudtemp2=pcmerge(FinalPointCloudtemp1,Sub2PtCloudFinal,0.1);
    FinalPointCloud=pcmerge(FinalPointCloudtemp2,Sub3PtCloudFinal,0.1);
     
% % %      p_temp=pcdenoise(FinalPointCloud,'NumNeighbors',300);
% % % 
% % %      p=pcdownsample(p_temp,'gridAverage',0.1);
% % %      [t,tnorm]=MyRobustCrust(p.Location);
% % %      figure
% % %      pcshow(p.Location,'b','VerticalAxis','Y','VerticalAxisDir','down')
% % % 
% % % figure
% % %         hold on
% % %         title('Output Triangulation','fontsize',14)
% % %         axis equal
% % %         trisurf(t,p.Location(:,1),p.Location(:,2),p.Location(:,3),'FaceColor','c','EdgeColor','b')
% % %         colormap('bone')
% % %      Surface_Lunge_Movie(k)=getframe(gcf);
     figure 
   hold on
   pcshow(MasterPtCloudFinal.Location,'b','VerticalAxis','Y','VerticalAxisDir','down')
   pcshow(Sub1PtCloudFinal.Location,'r','VerticalAxis','Y','VerticalAxisDir','down')
   pcshow(Sub2PtCloudFinal.Location,'g','VerticalAxis','Y','VerticalAxisDir','down')
   pcshow(Sub3PtCloudFinal.Location,'y','VerticalAxis','Y','VerticalAxisDir','down')
% %    xlim([-1000 1000])
% %     ylim([-1000 1000])
% %     zlim([2000 4000])
% %     view(90,0)

figure 
   hold on
   pcshow(FinalPointCloud.Location,'b','VerticalAxis','Y','VerticalAxisDir','down')
% % %    xlim([-1000 1000])
% % %     ylim([-1000 1000])
% % %     zlim([2000 4000])
% % %     view(90,0)
% %     set(gcf,'color','w');
% %     set(gca,'color','w');

            Lunge_Movie(k)=getframe(gcf);

            k=k+1;
%             close

            
end
end

% % % videoname = ('C:\Users\Abby.Eustace\Desktop\Kinect Azure\DataCollection\DataJan_23\LungeMotion.avi');
% % % 
% % %         v=VideoWriter(videoname);
% % % 
% % %         v.Quality=100;
% % % 
% % %         v.FrameRate=100;
% % % 
% % %         open(v)
% % % 
% % %         for i=1:length(Lunge_Movie)
% % % 
% % %             writeVideo(v,Lunge_Movie(i))
% % % 
% % %         end
% % % 
% % %         close(v)
% % % 
% % %     
