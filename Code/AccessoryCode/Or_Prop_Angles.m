%Initial orientation x, y, z (unsure which axis relates to which variable)
rotationAng1 = 0;
rotationAng2 = 0;
rotationAng3 = 0;

%Rotation rate x, y, z (unsure which axis relates to which variable)
omega1 = 0/2/1000;
omega2 = 0/2/1000;
omega3 = pi/2/1000;

rotations = 1000
%Convert the angle to quaternion (Default rotation sequence is ZYX...whatever rotation sequence means...)
%https://www.mathworks.com/help/aerotbx/ug/angle2quat.html
quaternion = angle2quat(rotationAng1,rotationAng2,rotationAng3);
transformedAngle = zeros(rotations,3);

for i=1:rotations
    %Get the quaternion that represents the rotation rate specified by omegas
    %https://books.google.com/books?id=crTwCAAAQBAJ&pg=PA559&lpg=PA559&dq=equations+for+attitude+propagation&source=bl&ots=J6Etc_UBs3&sig=i0radNUxOi_BLqEDsPwhup_8ypw&hl=en&sa=X&ved=0ahUKEwi3r93oiaPVAhVij1QKHSk-AjQQ6AEIMTAC#v=onepage&q=equations%20for%20attitude%20propagation&f=false
    %https://books.google.com/books?id=4xmlDgAAQBAJ&pg=PA614&lpg=PA614&dq=propagating+dynamics+quaternion&source=bl&ots=XUOBCK9vRC&sig=BHphjBWBlDskeCyeSJfjiXG7ceM&hl=en&sa=X&ved=0ahUKEwjj68Dy_qLVAhXLgVQKHRl-AC4Q6AEIVTAJ#v=onepage&q=propagating%20dynamics%20quaternion&f=false
    dQdt = 0.5 * [
            0,omega3,-omega2,omega1;
            -omega3,0,omega1,omega2; 
            omega2,-omega1,0,omega3;
            -omega1, -omega2, -omega3, 0;
            ]*transpose(quaternion);
        
    %Other source uses the following to find dQdt, where q3 is the scalar
    %term
    %http://www.vectornav.com/docs/default-source/documentation/vn-100-documentation/AN002.pdf?sfvrsn=19ee6b9_10
    q0 = quaternion(2);
    q1 = quaternion(3);
    q2 = quaternion(4);
    q3 = quaternion(1);
    %dQdt = 0.5 * [
    %    q0, q3, -q2, q1;
    %    q1, q2, q3, -q0;
    %    q2, -q1, q0, q3;
    %    q3, -q0, -q1, -q2;
    %    ] * transpose([0, omega1, omega2, omega3]);
    
    %Other source uses the following to get the update quaternion, but requires multiplication
    %https://stackoverflow.com/questions/23503151/how-to-update-quaternion-based-on-3d-gyro-data
    w = cos(omega1/2) * cos(omega2/2) * cos(omega3/2) + sin(omega1/2) * sin(omega2/2) * sin(omega3/2);
    x = sin(omega1/2) * cos(omega2/2) * cos(omega3/2) - cos(omega1/2) * sin(omega2/2) * sin(omega3/2);
    y = cos(omega1/2) * sin(omega2/2) * cos(omega3/2) + sin(omega1/2) * cos(omega2/2) * sin(omega3/2);
    z = cos(omega1/2) * cos(omega2/2) * sin(omega3/2) - sin(omega1/2) * sin(omega2/2) * cos(omega3/2);
    %dQdt = [w, x, y, z];
    %Get the quaternion that represents the new quaternion after applying the
    %rotation rate specified by the omegas, one timestep at a time.
    %https://www.mathworks.com/help/aerotbx/ug/quatmultiply.html
    %quaternion = quatmultiply(quaternion,(dQdt));
    quaternion = quaternion + transpose(dQdt);
    %Convert from quaternion back to angles
    %https://www.mathworks.com/help/aerotbx/ug/quat2angle.html
    [a,b,c]=quat2angle(quaternion);
    [i, i*omega2];    
    transformedAngle(i,:) = [a*57.2958, b*57.2958, c*57.2958];
end
transformedAngle(rotations,:)
plot(transformedAngle(:,3))


%%Notes
%Flight simulator thing -- https://www.mathworks.com/help/aerotbx/ug/using-aero-animation-objects.html
%Flight simulator thing API Help -- https://www.mathworks.com/help/aerotbx/ug/playaero.flightgearanimation.html
%Other axes transformations -- https://www.mathworks.com/help/aerotbx/axes-transformations-1.html
%The idiot who came up with this rotation system thing -- https://en.wikipedia.org/wiki/Euler_angles#Tait.E2.80.93Bryan_angles
%Converting Euler to/from Quat -- http://quat.zachbennett.com/
%Multiplying = converting one quaternion by the rotation specified by
%another quaternion -- https://stackoverflow.com/questions/30155403/how-do-you-rotate-a-quaternion-a-specified-angle-around-the-y-axis
%Intro to quaternions -- http://www.tu-berlin.de/fileadmin/fg169/miscellaneous/Quaternions.pdf
%Others
%http://www.chrobotics.com/library/understanding-euler-angles
%https://math.stackexchange.com/questions/773902/integrating-body-angular-velocity
%http://danceswithcode.net/engineeringnotes/rotations_in_3d/demo3D/rotations_in_3d_tool.html
%https://www.mathworks.com/help/physmod/sm/mech/gs/representations-of-body-orientation.html
%https://www.mathworks.com/help/aerotbx/axes-transformations.html
%https://math.stackexchange.com/questions/261193/updating-a-quaternion-orientation-by-a-vector-of-euler-angles
%http://stanford.edu/class/ee267/lectures/lecture10.pdf
%http://x-io.co.uk/x-imu/
%http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
%Very good resource: http://ai.stanford.edu/~varung/rigidbody.pdf
%http://butikov.faculty.ifmo.ru/Applets/Precession.html#_applet
%https://stackoverflow.com/questions/29229611/quaternion-to-axis-angles
%http://www.newocr.com/


if(1==2)
    %% Overlaying Simulated and Actual Flight Data
    % This example shows how to Visualize simulated versus actual flight 
    % trajectories with the animation object (Aero.Animation) while showing some 
    % of the animation object functionality. In this example, you can use the 
    % Aero.Animation object to create and configure an animation object, 
    % then use that object to create, visualize, and manipulate bodies for the 
    % flight trajectories.
    %
    % Copyright 1990-2012 The MathWorks, Inc.

    %% Create the Animation Object
    % This code creates an instance of the |Aero.Animation| object. 
    clear all
    h = Aero.Animation;

    %% Set the Animation Object Properties

    %%
    % This code sets the number of frames per second. This controls the rate at
    % which frames are displayed in the figure window.
    h.FramesPerSecond = 1;

    %%
    % This code sets the seconds of animation data per second time scaling.
    % This property and the |'FramesPerSecond'| property determine the time
    % step of the simulation. The settings in this example result in a time step
    % of approximately 0.5s. The equation is (1/FramesPerSecond)*TimeScaling
    % along with some extra terms to handle for sub-second precision.
    h.TimeScaling = 1;

    %% Create and Load Bodies
    % This code loads the bodies using
    % |createBody| for the animation object, |h|. This example will use these
    % bodies to work with and display the simulated and actual flight
    % trajectories. The first body is orange and will represent simulated data.
    % The second body is blue and will represent the actual flight data.
    idx1 = h.createBody('pa24-250_orange.ac','Ac3d');

    %% Load Recorded Data for Flight Trajectories
    % Using the bodies from the previous code, this code provides simulated and
    % actual recorded data for flight trajectories in the following files:
    %
    % * The simdata file contains logged simulated data. |simdata| is set up as a 6DoF
    % array, which is one of the default data formats.
    %
    % * The fltdata file contains actual flight test data. In this example, |fltdata| is
    % set up in a custom format. The example must create a custom read function
    % and set the |'TimeSeriesSourceType'| parameter to |'Custom'|.
    % 
    % To load the simdata and fltdata files: 
    load simdata

    %%
    % To work with the custom flight test data, this code sets the second body
    % |'TimeSeriesReadFcn'|. The custom read function is located here:
    % |<matlab:edit([matlabroot,'/toolbox/aero/astdemos/CustomReadBodyTSData.m']) matlabroot/toolbox/aero/astdemos/CustomReadBodyTSData.m>|


    %%
    % Set the bodies' timeseries data.
    h.Bodies{1}.TimeSeriesSource = simdata;

    %% Display Body Geometries in Figure
    % This code uses the |show| method to
    % create the figure graphics object for the animation object.
    h.show();

    %% Use the Animation Object to Play Back Flight Trajectories
    % This code uses the |play| method to animate bodies for the duration of the timeseries data.
    % Using this method will illustrate the slight differences between the
    % simulated and flight data.
    h.play();

    %% Camera Manipulation
    % This code illustrates how you can manipulate the camera for the two
    % bodies.
    %
    % The |'PositionFcn'| property of a camera object controls the camera
    % position relative to the bodies in the animation. The default camera
    % |'PositionFcn'| follows the path of a first order chase vehicle.
    % Therefore, it takes a few steps for the camera to position itself
    % correctly in the chase plane position.
    %
    % The default |'PositionFcn'| is here: 
    % |<matlab:edit([matlabroot,'/toolbox/aero/aero/@Aero/@Camera/private/doFirstOrderChaseCameraDynamics.m']) matlabroot/toolbox/aero/aero/@Aero/@Camera/private/doFirstOrderChaseCameraDynamics.m>|
    %
    % The default |'PositionFcn'| was used in the preceding animation playback.

    %%
    % The code can also use a custom, simplified |'PositionFcn'| that is a static
    % position based on the position of the bodies (i.e., no dynamics). The
    % simplified |'PositionFcn'| is located here:
    % |<matlab:edit([matlabroot,'/toolbox/aero/astdemos/staticCameraPosition.m']) matlabroot/toolbox/aero/astdemos/staticCameraPosition.m>|
    %
    % Set the new |'PositionFcn'|.
    %h.Camera.PositionFcn = @staticCameraPosition;

    %%
    % Run the animation with new |'PositionFcn'|.
    %h.play();

    %% Move Bodies
    % This code illustrates how to move the bodies to the starting position
    % (based on timeseries data) and update the camera position according to
    % the new |'PositionFcn'|. This code uses |updateBodies| and |updateCamera|.
    % t = 0;
    % h.updateBodies(t);
    % h.updateCamera(t);

    %% Reposition Bodies
    % This code illustrates how to reposition the bodies by first getting the
    % current body position and then separating the bodies.

    %%
    % Get current body positions and rotations from the body objects.
    % pos1 = h.Bodies{1}.Position;
    % rot1 = h.Bodies{1}.Rotation;

    %%
    % Separate bodies using |moveBody|.
    % This code separates and repositions the two bodies.
    % h.moveBody(1,pos1 + [0 0 -3],rot1);

    %% Create Transparency in the First Body
    % This code illustrates how to create transparency in the first body. The
    % code does this by changing the body patch properties via
    % |'PatchHandles'|. (For more information on patches in MATLAB(R), see the
    % |<matlab:helpview([docroot,'/techdoc/visualize/visualize.map'],'creating_3-d_models_with_patches') Creating 3-D Models with Patches>| section in the MATLAB documentation.)
    %
    % Note: On some platforms utilizing software OpenGL(R) rendering, the
    % transparency may cause a decrease in animation speed.
    %
    % See the |opengl| documentation for more information on OpenGL in MATLAB.

    %%
    % To create a transparency, the code gets the patch handles for the first
    % body.
    %patchHandles2 = h.Bodies{1}.PatchHandles;

    %%
    % Set desired face and edge alpha values.
    %desiredFaceTransparency = .3;
    %desiredEdgeTransparency = 1;
    %% Close and Delete Animation Object
    % To close and delete
    %h.delete();

    %%
    %displayEndOfDemoMessage(mfilename)
end
