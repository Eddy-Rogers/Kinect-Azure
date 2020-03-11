clc;clear all;close all;

fname = 'C:\Users\Abby.Eustace\Desktop\Kinect Azure\Cob Kinect Azure\x64\Debug\DUPELV03_06.json'; % Need to define file path
val = jsondecode(fileread(fname));

for framei=1:length(val.frames)
    JointData(:,:,framei)=val.frames(framei).bodies.joint_positions;
    
    figure
    plot(JointData([1:4 19:26],1,framei),-JointData([1:4 19:26],2,framei),'o')
    Squat_Movie(framei)=getframe(gcf);
     
end

videoname = ('C:\Users\Abby.Eustace\Desktop\Kinect Azure\DataCollection\DUPelv\S03\Trials\SquatJoints.avi');

        v=VideoWriter(videoname);

        v.Quality=100;

        v.FrameRate=10;

        open(v)

        for i=1:length(Squat_Movie)

            writeVideo(v,Squat_Movie(i))

        end

        close(v)
