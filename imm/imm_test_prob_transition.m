num_modes = 12;    % 

% probability of each mode
prob = zeros(num_modes,1); prob(5) = 1.0;

% the threshold to convert foot force to binary contact 
foot_forces_thres = [50,50,50,50];


% from foot force, generate transition probablitity
foot_forces = foot_force.Data(100,:);
contact_flags = foot_forces > foot_forces_thres;
% convert binary contact list to contact mode
flag_mode = bi2de(contact_flags) + 1;  % range (1-12)

% transition matrix 
H = 0.1/(num_modes-1)*ones(num_modes, num_modes);
trans_prob = 0.9;
H(flag_mode,:) = trans_prob;

% test transition
a = rand(num_modes,1);a = a/sum(a)

H*a