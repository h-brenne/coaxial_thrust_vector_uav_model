
rotor = rigidBodyTree("DataFormat","column");
base = rotor.Base;

hub = rigidBody("hub");
teetering_hub = rigidBody("teetering_hub");
blade_side_hub_positive = rigidBody("blade_side_hub_positive");
blade_side_hub_negative = rigidBody("blade_side_hub_negative");

jnt_hub = rigidBodyJoint("hub_joint","revolute");
jnt_teeter = rigidBodyJoint("jnt_teeter","revolute");
jnt_lag_pitch_positive = rigidBodyJoint("jnt_lag_pitch_positive","revolute");
jnt_lag_pitch_negative = rigidBodyJoint("jnt_lag_pitch_negative","revolute");

jnt_teeter.JointAxis = [0 1 0];
jnt_lag_pitch_positive.JointAxis = [0.5 0 0.5];
jnt_lag_pitch_negative.JointAxis = [0.5 0 0.5];

% Teetering heigth over hub
setFixedTransform(jnt_teeter,trvec2tform([0 0 0.2]));
setFixedTransform(jnt_lag_pitch_positive,trvec2tform([0.15 0 0]));
setFixedTransform(jnt_lag_pitch_negative,trvec2tform([-0.15 0 0]));

% Assemble

hub.Joint = jnt_hub;
teetering_hub.Joint = jnt_teeter;
blade_side_hub_positive.Joint = jnt_lag_pitch_positive;
blade_side_hub_negative.Joint = jnt_lag_pitch_negative;

addBody(rotor, hub, base.Name);
addBody(rotor, teetering_hub, hub.Name);
addBody(rotor, blade_side_hub_positive, teetering_hub.Name);
addBody(rotor, blade_side_hub_negative, teetering_hub.Name);

figure("Name","Assemble Robot","Visible","on")
showdetails(rotor)
show(rotor);
drawnow;

figure("Name","Interactive GUI")
gui = interactiveRigidBodyTree(rotor,"MarkerScaleFactor",0.25);
