clc; clear all ;close all
MasterPointFilePath=dir('C:\Users\Abby.Eustace\Desktop\Kinect Azure\DataCollection\ReflectiveCal\PlyFiles\AposeMaster\*.ply');
Sub1PointFilePath=dir('C:\Users\Abby.Eustace\Desktop\Kinect Azure\DataCollection\ReflectiveCal\PlyFiles\AposeSub1\*.ply');
Sub2PointFilePath=dir('C:\Users\Abby.Eustace\Desktop\Kinect Azure\DataCollection\ReflectiveCal\PlyFiles\AposeSub2\*.ply');
Sub3PointFilePath=dir('C:\Users\Abby.Eustace\Desktop\Kinect Azure\DataCollection\ReflectiveCal\PlyFiles\AposeSub3\*.ply');

MasterFileLength=length(MasterPointFilePath);
Sub1FileLength=length(Sub1PointFilePath);
Sub2FileLength=length(Sub2PointFilePath);
Sub3FileLength=length(Sub3PointFilePath);

LeastFiles=min([MasterFileLength,Sub1FileLength,Sub2FileLength,Sub3FileLength]);

TransSub1Master=[0.004808995873, 0.160664588213, -0.986997365952, 2023.349365234375;
-0.115519970655, 0.980489969254, 0.159042447805, -362.736145019531;
0.993293523788, 0.113253071904, 0.023275140673, 2783.135253906250;
0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000];

TransSub2Master=[-0.998836696148, 0.035002190620, -0.033167019486, 322.886474609375;
0.026599202305, 0.973652303219, 0.226481065154, -570.274169921875;
0.040220480412, 0.225335389376, -0.973450720310, 5336.599609375000;
0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000];

TransSub3Master=[-0.070497132838, -0.089819639921, 0.993459880352, -1707.034790039063;
0.090192914009, 0.991284430027, 0.096023149788, -207.642150878906;
-0.993426084518, 0.096372403204, -0.061781611294, 2603.811767578125;
0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000];

k=1;
for countFile=10%1:LeastFiles
    
if exist([MasterPointFilePath(1).folder,'\',sprintf('AposeCalReflectMaster.%d.ply',countFile)])==2 && ...  
exist([Sub1PointFilePath(1).folder,'\',sprintf('AposeCalReflectSub1.%d.ply',countFile)])==2 && ...  
exist([Sub2PointFilePath(1).folder,'\',sprintf('AposeCalReflectSub2.%d.ply',countFile)])==2 && ...  
exist([Sub3PointFilePath(1).folder,'\',sprintf('AposeCalReflectSub3.%d.ply',countFile)])==2
    
MasterPointFile=[MasterPointFilePath(1).folder,'\',sprintf('AposeCalReflectMaster.%d.ply',countFile)];  
Sub1PointFile=[Sub1PointFilePath(1).folder,'\',sprintf('AposeCalReflectSub1.%d.ply',countFile)];  
Sub2PointFile=[Sub2PointFilePath(1).folder,'\',sprintf('AposeCalReflectSub2.%d.ply',countFile)];  
Sub3PointFile=[Sub3PointFilePath(1).folder,'\',sprintf('AposeCalReflectSub3.%d.ply',countFile)];  


MasterPtCloud=pcread(MasterPointFile);
    Sub1PtCloud=pcread(Sub1PointFile);
      Sub2PtCloud=pcread(Sub2PointFile);
        Sub3PtCloud=pcread(Sub3PointFile);
    
    LengthSub1=length(Sub1PtCloud.Location);
     LengthSub2=length(Sub2PtCloud.Location);
      LengthSub3=length(Sub3PtCloud.Location);
    
    Sub1PtCloudTemp=ones(LengthSub1,4);
    Sub1PtCloudTemp(:,1:3)=Sub1PtCloud.Location(:,:);
 
     Sub2PtCloudTemp=ones(LengthSub2,4);
    Sub2PtCloudTemp(:,1:3)=Sub2PtCloud.Location(:,:);
    
     Sub3PtCloudTemp=ones(LengthSub3,4);
    Sub3PtCloudTemp(:,1:3)=Sub3PtCloud.Location(:,:);
    
   for s1=1:LengthSub1
       Sub1PtCloudTrans(s1,:)=TransSub1Master*Sub1PtCloudTemp(s1,:)';
   end
    
   for s2=1:LengthSub2
       Sub2PtCloudTrans(s2,:)=TransSub2Master*Sub2PtCloudTemp(s2,:)';
   end
   
   for s3=1:LengthSub3
       Sub3PtCloudTrans(s3,:)=TransSub3Master*Sub3PtCloudTemp(s3,:)';
   end
   Sub1PtCloudFinal=Sub1PtCloudTrans(:,1:3);
    Sub2PtCloudFinal=Sub2PtCloudTrans(:,1:3);
     Sub3PtCloudFinal=Sub3PtCloudTrans(:,1:3);
     
   figure 
   hold on
   pcshow(MasterPtCloud.Location,'b','VerticalAxis','Y','VerticalAxisDir','down')
   pcshow(Sub1PtCloudFinal,'b','VerticalAxis','Y','VerticalAxisDir','down')
   pcshow(Sub2PtCloudFinal,'b','VerticalAxis','Y','VerticalAxisDir','down')
   pcshow(Sub3PtCloudFinal,'b','VerticalAxis','Y','VerticalAxisDir','down')
   xlim([-1000 1000])
    ylim([-1000 1000])
    zlim([2000 4000])

    set(gcf,'color','w');
    set(gca,'color','w');

            Lunge_Movie(k)=getframe(gcf);

            k=k+1;
%             close
            
end
end

% % % videoname = ('C:\Users\Abby.Eustace\Desktop\ReflectiveCal\LungeMotion.avi');
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

    