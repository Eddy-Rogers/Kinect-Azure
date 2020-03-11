clc; clear all ;close all
MasterPointFilePath=dir('C:\Users\Abby.Eustace\Desktop\Kinect Azure\DataCollection\DUPelv\S03\Trials\plyFiles\Master\*.ply');
Sub1PointFilePath=dir('C:\Users\Abby.Eustace\Desktop\Kinect Azure\DataCollection\DUPelv\S03\Trials\plyFiles\Sub\*.ply');

MasterFileLength=length(MasterPointFilePath);
Sub1FileLength=length(Sub1PointFilePath);

LeastFiles=min([MasterFileLength,Sub1FileLength]);

% % % angle=1.3*(pi/180);
% % % WFOV_NFOVTrans=[1 0 0 0;
% % %     0 cos(angle) -sin(angle)  0;
% % %     0 sin(angle) cos(angle)  0;
% % %     0 0 0 1];

TransSub1Master=[0.319791764021 0.126292467117 -0.939033269882 1303.155273437500;
-0.099132902920 0.990097105503 0.099400021136 -120.232543945313;
0.942287564278 0.061301779002 0.329144656658 803.105102539063;
0.000000000000 0.000000000000 0.000000000000 1.000000000000];



k=1;

for countFile=1:25%1:LeastFiles
    
if exist([MasterPointFilePath(1).folder,'\',sprintf('Master_DUPELV03_02.%d.ply',countFile)])==2 && ...  
exist([Sub1PointFilePath(1).folder,'\',sprintf('Sub_DUPELV03_02.%d.ply',countFile)])==2 
    
MasterPointFile=[MasterPointFilePath(1).folder,'\',sprintf('Master_DUPELV03_02.%d.ply',countFile)];  
Sub1PointFile=[Sub1PointFilePath(1).folder,'\',sprintf('Sub_DUPELV03_02.%d.ply',countFile)];  

MasterPtCloud=pcread(MasterPointFile);
    Sub1PtCloud=pcread(Sub1PointFile);
  figure    
  pcshow(MasterPtCloud.Location)
  figure
  pcshow(Sub1PtCloud.Location)
  
    LengthMaster=length(MasterPtCloud.Location);
    LengthSub1=length(Sub1PtCloud.Location);
     
         MasterPtCloudTemp=ones(LengthMaster,4);
    MasterPtCloudTemp(:,1:3)=MasterPtCloud.Location(:,:);
    
    Sub1PtCloudTemp=ones(LengthSub1,4);
    Sub1PtCloudTemp(:,1:3)=Sub1PtCloud.Location(:,:);
 
     
  
       MasterPtCloudTrans=MasterPtCloudTemp;
  
    
   for s1=1:LengthSub1
       Sub1PtCloudTrans(s1,:)=TransSub1Master*Sub1PtCloudTemp(s1,:)';
   end
    
   
      MasterPtCloudFinal=pointCloud(MasterPtCloudTrans(:,1:3));
   Sub1PtCloudFinal=pointCloud(Sub1PtCloudTrans(:,1:3));
    
     
     FinalPointCloud=pcmerge(MasterPtCloudFinal,Sub1PtCloudFinal,0.1);
   
     
% %      p_temp=pcdenoise(FinalPointCloud,'NumNeighbors',300);
% % 
% %      p=pcdownsample(p_temp,'gridAverage',0.1);
% %      [t,tnorm]=MyRobustCrust(p.Location);
% %      figure
% %      pcshow(p.Location,'b','VerticalAxis','Y','VerticalAxisDir','down')
% % 
% % figure
% %         hold on
% %         title('Output Triangulation','fontsize',14)
% %         axis equal
% %         trisurf(t,p.Location(:,1),p.Location(:,2),p.Location(:,3),'FaceColor','c','EdgeColor','b')
% %         colormap('bone')
% %      Surface_Lunge_Movie(k)=getframe(gcf);
% %      figure 
% %    hold on
% %    pcshow(MasterPtCloudFinal,'b','VerticalAxis','Y','VerticalAxisDir','down')
% %    pcshow(Sub1PtCloudFinal,'b','VerticalAxis','Y','VerticalAxisDir','down')
% %    pcshow(Sub2PtCloudFinal,'b','VerticalAxis','Y','VerticalAxisDir','down')
% %    pcshow(Sub3PtCloudFinal,'b','VerticalAxis','Y','VerticalAxisDir','down')
% %    xlim([-1000 1000])
% %     ylim([-1000 1000])
% %     zlim([2000 4000])
% %     view(90,0)
% % % % 
figure 
   hold on
   pcshow(FinalPointCloud.Location,'VerticalAxis','Y','VerticalAxisDir','down')
   xlim([-500 500])
    ylim([-500 1000])
    zlim([750 1700])
    colormap
% % % %     view(90,0)
% %     set(gcf,'color','w');
% %     set(gca,'color','w');

            T02_Movie(k)=getframe(gcf);

            k=k+1;
%             close

            
end
end

videoname = ('C:\Users\Abby.Eustace\Desktop\Kinect Azure\DataCollection\DUPelv\S03\Trials\plyFiles\SeatedSlouched.avi');

        v=VideoWriter(videoname);

        v.Quality=100;

        v.FrameRate=10;

        open(v)

        for i=1:length(T02_Movie)

            writeVideo(v,T02_Movie(i))

        end

        close(v)

    
