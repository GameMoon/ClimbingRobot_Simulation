%Init
robot = importrobot("climbingrobot_exported.urdf");
robot.DataFormat = 'column';
hConfig = homeConfiguration(robot);


defaultAngle = pi/8;
for i = 1:3:12
    hConfig(i) = 0;
    hConfig(i+1) = defaultAngle;
    hConfig(i+2) = -defaultAngle*0.9;
end
initConfig = hConfig;

if any(contains(robot.BodyNames,"feet1"))
    removeBody(robot,"feet1");
    removeBody(robot,"feet2");
    removeBody(robot,"feet3");
    removeBody(robot,"feet4");
end

foot1 = rigidBody("feet1");
jnt1 = rigidBodyJoint('jnt1');
setFixedTransform(jnt1,trvec2tform([0,0,-0.026125/2]));
foot1.Joint = jnt1;

foot2 = rigidBody("feet2");
jnt2 = rigidBodyJoint('jnt2');
setFixedTransform(jnt2,trvec2tform([0,0,-0.026125/2]));
foot2.Joint = jnt2;

foot3 = rigidBody("feet3");
jnt3 = rigidBodyJoint('jnt3');
setFixedTransform(jnt3,trvec2tform([0,0,-0.026125/2]));
foot3.Joint = jnt3;

foot4 = rigidBody("feet4");
jnt4 = rigidBodyJoint('jnt4');
setFixedTransform(jnt4,trvec2tform([0,0,-0.026125/2]));
foot4.Joint = jnt4;

addBody(robot,foot1,"magnet11");
addBody(robot,foot2,"magnet21");
addBody(robot,foot3,"magnet31");
addBody(robot,foot4,"magnet41");

ik = inverseKinematics('RigidBodyTree',robot);
weights = [0 0 0 1 1 1];

% sim constants
contact_stiffness = 400/0.001;          % Approximated at weight (N) / desired displacement (m)
contact_damping = contact_stiffness/10; % Tuned based on contact stiffness value
mu_s = 0.9;     % Static friction coefficient: Around that of rubber-asphalt
mu_k = 0.8;     % Kinetic friction coefficient: Lower than the static coefficient
mu_vth = 0.1;   % Friction velocity threshold (m/s)