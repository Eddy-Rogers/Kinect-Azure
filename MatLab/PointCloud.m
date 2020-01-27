% % % ptCloud = pcread('Example.ply');
% % % pcshow(ptCloud);
clc; clear all ;close all
% % SubjectPath = uigetdir('C:\','Select the main folder where all files are');
PlyFiles=dir('C:\Users\Abby.Eustace\Desktop\Kinect Azure\DataCollection\MarkersEffect\*.ply');

% % PlyFiles=dir(fullfile(SubjectPath,'\PLY_Files\','*.ply'));
% % JsonFiles=dir(fullfile(SubjectPath,'\PLY_Files\','*.json'));
 Filename='Frame';%PlyFiles(1,1).name(1:5);
 
% val=(jsondecode(fileread([SubjectPath,'\PLY_Files\Subject07.Trial01.json'])));%,JsonFiles(4,1).name])));
for ii=10%75:100%1:length(PlyFiles)
%     ptCloud=pcread([SubjectPath,'\PLY_Files\',Filename,sprintf('%d.ply',ii-1)]);
    ptCloud=pcread(['C:\Users\Abby.Eustace\Desktop\Kinect Azure\DataCollection\MarkersEffect\',PlyFiles(ii).name]);
% %     figure
% %     hold on
% %     pcshow(ptCloud);
% %     plot3(val.frames(ii).bodies.joint_positions(1:27,1),val.frames(ii).bodies.joint_positions(1:27,2),val.frames(ii).bodies.joint_positions(1:27,3),'x')   
    xlabel('X(m)')
    ylabel('Y(m)')
    zlabel('Z(m)')

% % % %% Isolating the body point cloud with clustering
% % % minDistance=20;
% % % [labels,numClusters]=pcsegdist(ptCloud,minDistance);
% % % 
% % % figure
% % % pcshow(ptCloud.Location,labels);
%% Preprocessing Point Cloud Filtering Out Background and Sides
BackgroundIndx=find(ptCloud.Location(:,3)<3900);%ptCloud.ZLimits(2)-1000);
FilBackPtCloud=ptCloud.Location(BackgroundIndx,:);
SidesIndx=find(FilBackPtCloud(:,1)<1000 & FilBackPtCloud(:,1)>-1000);
SidePtCloud=FilBackPtCloud(SidesIndx,:);
TopIndx=find(SidePtCloud(:,2)>-2000);
FinalPtCloud=SidePtCloud(TopIndx,:);
% figure 
% pcshow(FinalPtCloud)
%% Understanding the distrabution of the points
% % % figure
% % % hx=histogram(ptCloud.Location(:,1));
% % % figure
% % % hy=histogram(ptCloud.Location(:,2));
% % % figure
% % % hz=histogram(ptCloud.Location(:,3));
[Z_N,Z_edges] = histcounts(FinalPtCloud(:,3));

[~, ZBinMaxIndx]=max(Z_N);
UpperZLimit=Z_edges(ZBinMaxIndx+1);
LowerZLimit=Z_edges(ZBinMaxIndx);
FilteredptCloudIndx=find(FinalPtCloud(:,3)<UpperZLimit+500);
Init_ptCloudIso=FinalPtCloud(FilteredptCloudIndx,:);
figure 
hold on
pcshow(Init_ptCloudIso,'b','VerticalAxis','Y','VerticalAxisDir','Down')
% plot3(val.frames(ii).bodies.joint_positions(1:27,1),val.frames(ii).bodies.joint_positions(1:27,2),val.frames(ii).bodies.joint_positions(1:27,3),'x')   

% % % %% Isolating the body point cloud with nearest neighbor 
% % % % point is the origin of starting the point search
% % % % K is the number of points to find around the origin
% % %     point=[0,-250,3500];
% % %     K=6000;
% % %     [indices,dists] = findNearestNeighbors(ptCloud,point,K);
% % % % Selecting only the isolated points from the larger point cloud 
% % %     ptCloudIso=select(ptCloud,indices);
% % %     
% % %     figure
% % %     hold on
% % %     pcshow(ptCloudIso.Location,'y');
% % %    
    
% % %     ptCloudIsoData=double(ptCloudIso.Location);
   
% % %     %% Find 3D delaunay triangles
% % % x=ptCloudIsoData(:,1)';
% % % y=ptCloudIsoData(:,2)';
% % % z=ptCloudIsoData(:,3)';
% % % 
% % % 
% % % 
% % % [~ ,CenterIndx]=min(ptCloudIsoData(:,3));
% % % Center=ptCloudIsoData(CenterIndx,:);
% % % 
% % % figure
% % % hold on
% % % pcshow(ptCloudIso.Location,'y');
% % % plot3(Center(1),Center(2),Center(3),'r*')
% % % 
% % % 
% % % LowerBodyIndx=find(ptCloudIsoData(:,2)>Center(2));
% % % UpperBodyIndx=find(ptCloudIsoData(:,2)<Center(2));
% % % 
% % % LowerBody=ptCloudIsoData(LowerBodyIndx,:);
% % % UpperBody=ptCloudIsoData(UpperBodyIndx,:);
% % % 
% % % figure
% % % pcshow(LowerBody)
% % % 
% % % figure
% % % pcshow(UpperBody)

% % % %% Body Classifications
% % % [~, SortUpIndx]=sort(UpperBody(:,2));
% % % SortedUpperBody=UpperBody(SortUpIndx,:);
% % % SortedUpperBody=round(SortedUpperBody);
% % % 
% % % UpperYValues=unique(SortedUpperBody(:,2));
% % % BodySlice={};
% % % for ss=1:length(UpperYValues)
% % %     count=1;
% % % for s=1:size(SortedUpperBody,1)
% % %    
% % %     if SortedUpperBody(s,2)==UpperYValues(ss)
% % %         BodySlice{s}(count,:)=SortedUpperBody(s,:);
% % %         count=count+1;
% % %     end
% % % end
% % % end
% % % 
% % % figure
% % % pcshow(Slice)

% % %     
% % %     Tri=delaunayTriangulation(ptCloudIsoData(:,1),ptCloudIsoData(:,2),ptCloudIsoData(:,3));
% % %     
% % %     trimesh(Tri.ConnectivityList,ptCloudIsoData(:,1),ptCloudIsoData(:,2),ptCloudIsoData(:,3))
% % %     
% % %     %% Creating edges 
% % %     delta=0.1;
% % %     p=1;
% % %     for nn=1:length(ptCloudIsoData)-1
% % %         Vert3D=[ptCloudIsoData(nn,:);ptCloudIsoData(nn+1,:)];
% % %         Vert2D=[ptCloudIsoData(nn,1:2);ptCloudIsoData(nn+1,1:2)];
% % %         SecondNorm=norm(ptCloudIsoData(nn+1,:)-ptCloudIsoData(nn,:));
% % %         InfNorm=max(abs(Vert2D(2,:)-Vert2D(1,:)));
% % %         Comp=delta^(InfNorm);
% % %         
% % %         if Comp > SecondNorm && Comp <= 1
% % %             Edge(p,:)=cross(ptCloudIsoData(nn,:),ptCloudIsoData(nn+1,:));
% % %             w(p)=SecondNorm;
% % %             p=p+1;
% % %         end
% % %     end
    
% % %     figure
% % %     plot(Edge)
% % %     roi = [-1000 1000 -2000 1000 -500 6000];
% % %     indices = findPointsInROI(ptCloud,roi);
% % %     ptCloudB = select(ptCloud,indices);
% % %     
% % %     figure
% % %     pcshow(ptCloud.Location,[0.5 0.5 0.5])
% % %     hold on
% % %     pcshow(ptCloudB.Location,'r');
    
% % %     minDistance=0.5;
% % %     
% % %     [labels,numClusters] = pcsegdist(ptCloud,minDistance);
% % %     pcshow(ptCloud.Location,labels)
% % %     colormap(hsv(numClusters))
    
% % %     ptCloudB = pcdenoise(ptCloud);
% % %     figure
% % %     pcshow(ptCloudB);
% % %  
end

