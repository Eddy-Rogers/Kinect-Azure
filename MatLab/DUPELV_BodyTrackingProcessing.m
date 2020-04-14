 clc; clear all; 
 
mkvPath = ('D:\DUPelv\S05\mvkFiles\'); 
% % % Path = ('C:\Users\Abby.Eustace\Desktop\Kinect Azure\Azure Kinect SDK v1.3.0\tools\Variability\');
exePath=('C:\Users\Abby.Eustace\Desktop\Kinect Azure\Cob Kinect Azure\x64\Debug\');
File={'01','02','03','04','05','06','07','08','09','10','11','12'};

FileName='Master_DUPELV05_';
for filei=1:length(File)
exe='OnlyJointProcessing.exe';
      jsonFile=[FileName File{filei} '.json'];
       mkvFile=[FileName File{filei} '.mkv'];
       
%% Process the mkv to get the Key Points
        system(['"',exePath,exe,'" "',mkvPath,mkvFile,'" "',mkvPath,jsonFile,'"']);

        pause(2)
        
end