 clc; clear all; 
 
mkvPath = ('D:\DUPelv\S04'); 
TransPath = ('D:\DUPelv\Cal(Mar5)\CalMar5.txt');
exePath=('C:\Users\Abby.Eustace\Documents\Eddy Github\Kinect-Azure-Processing\out\build\x64-Debug\');
File={'T01','T02','T03','T04','T05','T06','T07','T08','T09','T10','T11'};

for filei=6:length(File)
exe='project.exe';
% %        MastermkvFile=sprintf('Master_DUPELV04_%s.mkv',File(filei));
% %        SubmkvFile=sprintf('Sub_DUPELV04_%s.mkv',File(filei));
       %% Process mkv Files to get ply Files
        system(['"',exePath,exe,'" "',TransPath,'" "',mkvPath,'\',File{filei},'"']);
       
end