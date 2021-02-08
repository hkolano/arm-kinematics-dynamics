% dhparams in order: [a alpha d theta]
% lengths now in m
theta_a = atan2(145.3, 40)
dhparams = [.020   	pi/2	.0462   pi;
           .15071	pi      0       -theta_a;
            .020    -pi/2	0   	-theta_a;
            0   	pi/2	-.180	pi/2;
            0       0       0   	-pi/2];
alphaArm = rigidBodyTree;

link1 = rigidBody('link1');
jntE = rigidBodyJoint('jntE','revolute');
jntE.HomePosition = pi;
jntE.PositionLimits = [5*pi/180, 350*pi/180];
link2 = rigidBody('link2');
jntD = rigidBodyJoint('jntD','revolute');
jntD.HomePosition = -theta_a;
jntD.PositionLimits = [-theta_a, -theta_a+200*pi/180];
link3 = rigidBody('link3');
jntC = rigidBodyJoint('jntC','revolute');
jntC.HomePosition = -theta_a;
jntC.PositionLimits = [-theta_a, -theta_a + 200*pi/180];
link4 = rigidBody('link4');
jntB = rigidBodyJoint('jntB','revolute');
jntB.HomePosition = pi/2;
jntB.PositionLimits = [-175*pi/180, 175*pi/180];
link5 = rigidBody('link5');
jntA = rigidBodyJoint('jntA','revolute');
jntA.HomePosition = -pi/2;

setFixedTransform(jntE,dhparams(1,:),'dh');
setFixedTransform(jntD,dhparams(2,:),'dh');
setFixedTransform(jntC,dhparams(3,:),'dh');
setFixedTransform(jntB,dhparams(4,:),'dh');
setFixedTransform(jntA,dhparams(5,:),'dh');

link1.Joint = jntE;
link2.Joint = jntD;
link3.Joint = jntC;
link4.Joint = jntB;
link5.Joint = jntA;

addBody(alphaArm,link1,'base')
addBody(alphaArm,link2,'link1')
addBody(alphaArm,link3,'link2')
addBody(alphaArm,link4,'link3')
addBody(alphaArm,link5,'link4')

config = homeConfiguration(alphaArm);
newconfig = config;

%showdetails(robot)
fig1 = show(alphaArm);

h=findall(fig1); %finding all objects in figure
hlines=h.findobj('Type','Line'); %finding line object 
%editing line object properties
n=size(hlines);
for i=1:n
    hlines(i).LineWidth=2; %chanding line width
    %hlines(i).Color=[1 0 1];%changing line color
    hlines(i).Marker='o';%changing marker type
    hlines(i).MarkerSize=10; %changing marker size
end
