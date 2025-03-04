function SIM_multicopter(jsonStr)
clc
clearvars -except jsonStr
close all

% Physics of a multi copterprearm motors check frame class and type”

% load in the parameters for a frame
try
    % Step 1: Try to load data from the JSON file
    fileText = fileread(jsonStr);
    state = jsondecode(fileText);
    disp('Loaded state from JSON file successfully.');
catch ME
    disp('Failed to load JSON file.');
    disp(['Error message: ', ME.message]);
end


% Setup environmental conditions
state.environment.density = 1.225; % (kg/m^3)
state.gravity_mss = 9.80665; % (m/s^2)

% Setup the time step size for the Physics model
max_timestep = 1/50;

% define init and time setup functions
init_function = @init;
physics_function = @physics_step;

% setup connection
SITL_connector(state,init_function,physics_function,max_timestep);

end 
% Simulator model must take and return a structure with the felids: 
% gyro(roll, pitch, yaw) (radians/sec) body frame
% attitude(roll, pitch yaw) (radians)
% accel(north, east, down) (m/s^2) body frame
% velocity(north, east,down) (m/s) earth frame
% position(north, east, down) (m) earth frame 
% the structure can have any other felids required for the physics model

% init values
function state = init(state)
for i = 1:numel(state.copter.motors)
    state.copter.motors(i).rpm = 0;
    state.copter.motors(i).current = 0;
end
state.gyro = [0;0;0]; % (rad/sec)
state.dcm = diag([1,1,1]); % direction cosine matrix
state.attitude = [0;0;0]; % (radians) (roll, pitch, yaw)
state.accel = [0;0;0]; % (m/s^2) body frame
state.velocity = [0;0;0]; % (m/s) earth frame
state.position = [0;0;0]; % (m) earth frame
state.bf_velo = [0;0;0]; % (m/s) body frame
end

% Take a physics time step
function state = physics_step(pwm_in,state)

% Calculate the dropped battery voltage, assume current draw from last step
state.copter.battery.current = sum([state.copter.motors.current]);
state.copter.battery.dropped_voltage = state.copter.battery.voltage - state.copter.battery.resistance * state.copter.battery.current;
% state.copter.battery.dropped_voltage = state.copter.battery.voltage;

% Calculate the torque and thrust, assume RPM is last step value
for i = 1:numel(state.copter.motors)
    motor = state.copter.motors(i);

    % Calculate the throttle
    throttle = (pwm_in(motor.channel) - 1100) / 800;
    throttle = max(throttle,0);
    throttle = min(throttle,1);

    % effective voltage
    voltage = throttle * state.copter.battery.dropped_voltage;

    % Take the RPM from the last step to calculate the new
    % torque and current
    Kt = 1/(motor.electrical.kv * ( (2*pi)/60) );

    % rpm equation rearranged for current
    current = ((motor.electrical.kv * voltage) - motor.rpm) / ((motor.electrical.resistance + motor.esc.resistance) * motor.electrical.kv);
    torque = current * Kt;

    prop_drag = motor.prop.PConst * state.environment.density * (motor.rpm/60)^2 * motor.prop.diameter^5;

    w = motor.rpm * ((2*pi)/60); % convert to rad/sec

    w1 = w + ((torque-prop_drag) / motor.prop.inertia) * state.delta_t;

    rps = w1 * (1/(2*pi));

    % can never have negative rps
    rps = max(rps,0);

    % Calculate the thrust (with fudge factor!)
    thrust = 4 * motor.prop.TConst * state.environment.density * rps^2 * motor.prop.diameter^4;
    
    % calculate resulting moments
    moment_roll = thrust * motor.location(1);
    moment_pitch = thrust * motor.location(2);
    moment_yaw = -torque * motor.direction;

    % Update main structure
    state.copter.motors(i).torque = torque;
    state.copter.motors(i).current = current;
    state.copter.motors(i).rpm = rps * 60;
    state.copter.motors(i).thrust = thrust;
    state.copter.motors(i).moment_roll = moment_roll;
    state.copter.motors(i).moment_pitch = moment_pitch;
    state.copter.motors(i).moment_yaw = moment_yaw;
end

drag = sign(state.bf_velo) .* state.copter.cd .* state.copter.cd_ref_area .* 0.5 .* state.environment.density .* state.bf_velo.^2;

% Calculate the forces about the CG (N,E,D) (body frame)
force = [0;0;-sum([state.copter.motors.thrust])] - drag;
% fprintf("force: %f\n", force);
% estimate rotational drag
rotational_drag = 0.2 * sign(state.gyro) .* state.gyro.^2; % estimated to give a reasonable max rotation rate

% Update attitude, moments to rotational acceleration to rotational velocity to attitude
moments = [-sum([state.copter.motors.moment_roll]);sum([state.copter.motors.moment_pitch]);sum([state.copter.motors.moment_yaw])] - rotational_drag;
% fprintf("R:%d P:%d in%d\n",-sum([state.copter.motors.moment_roll]), sum([state.copter.motors.moment_pitch]), pwm_in(1:4));

state = update_dynamics(state,force,moments);

end

% integrate the acceleration resulting from the forces and moments to get the
% new state
function state = update_dynamics(state,force,moments)

rot_accel = (moments' / state.copter.inertia)';

state.gyro = state.gyro + rot_accel * state.delta_t;

% Constrain to 2000 deg per second, this is what typical sensors max out at
state.gyro = max(state.gyro,deg2rad(-2000));
state.gyro = min(state.gyro,deg2rad(2000));

% update the dcm and attitude
[state.dcm, state.attitude] = rotate_dcm(state.dcm,state.gyro * state.delta_t);

% body frame accelerations
state.accel = force / state.copter.mass;

% earth frame accelerations (NED)
accel_ef = state.dcm * state.accel;
accel_ef(3) = accel_ef(3) + state.gravity_mss;

% if we're on the ground, then our vertical acceleration is limited
% to zero. This effectively adds the force of the ground on the aircraft
if state.position(3) >= 0 && accel_ef(3) > 0
    accel_ef(3) = 0;
end

% work out acceleration as seen by the accelerometers. It sees the kinematic
% acceleration (ie. real movement), plus gravity
state.accel = state.dcm' * (accel_ef + [0; 0; -state.gravity_mss]);

state.velocity = state.velocity + accel_ef * state.delta_t;
state.position = state.position + state.velocity * state.delta_t;

% make sure we can't go underground (NED so underground is positive)
if state.position(3) >= 0
    state.position(3) = 0;
    state.velocity = [0;0;0];
    state.gyro = [0;0;0];
end

% calculate the body frame velocity for drag calculation
state.bf_velo = state.dcm' * state.velocity;

end

function [dcm, euler] = rotate_dcm(dcm, ang)

% rotate
delta = [dcm(1,2) * ang(3) - dcm(1,3) * ang(2),         dcm(1,3) * ang(1) - dcm(1,1) * ang(3),      dcm(1,1) * ang(2) - dcm(1,2) * ang(1);
         dcm(2,2) * ang(3) - dcm(2,3) * ang(2),         dcm(2,3) * ang(1) - dcm(2,1) * ang(3),      dcm(2,1) * ang(2) - dcm(2,2) * ang(1);
         dcm(3,2) * ang(3) - dcm(3,3) * ang(2),         dcm(3,3) * ang(1) - dcm(3,1) * ang(3),      dcm(3,1) * ang(2) - dcm(3,2) * ang(1)];

dcm = dcm + delta;

% normalise
a = dcm(1,:);
b = dcm(2,:);
error = a * b';
t0 = a - (b *(0.5 * error));
t1 = b - (a *(0.5 * error));
t2 = cross(t0,t1);
dcm(1,:) = t0 * (1/norm(t0));
dcm(2,:) = t1 * (1/norm(t1));
dcm(3,:) = t2 * (1/norm(t2));

% calculate euler angles
euler = [atan2(dcm(3,2),dcm(3,3)); -asin(dcm(3,1)); atan2(dcm(2,1),dcm(1,1))]; 

end

