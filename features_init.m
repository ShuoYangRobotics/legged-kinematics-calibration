param.num_features = 200;
R = 3;    % radius of the sphere
Xc = tgt_x;
Yc = tgt_y;
Zc = tgt_z;  % (Xc,Yc,Zc) center of the sphere

elevation = pi/10*(2*rand(param.num_features,1)-1);
azimuth = pi/4*(2*rand(param.num_features,1)-1);
radius = R + 0.5*(2*rand(param.num_features,1)-1);
[feature_x,feature_y,feature_z] = sph2cart(azimuth,elevation,radius);
param.feature_x=feature_x+Xc;
param.feature_y=feature_y+Yc;
param.feature_z=feature_z+Zc;