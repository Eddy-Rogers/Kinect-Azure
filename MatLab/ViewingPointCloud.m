ptCloud=pcread(['C:\Users\Abby.Eustace\Desktop\Kinect Azure\Cob Kinect Azure\x64\Debug\Marker Testing\t_pose_modified.ply']); % The square bracket park is the filepath to the .ply file
    figure
    c=pcshow(ptCloud);
%     set(c,'colormap',gray)
color=[1:-.01:0.4,0.4:0.01:1];
    colormap([color',color',color'])
    grid off
    axis off
    
%     pcwrite(ptCloud,'NewCameraFourFrame3Iso.ply')