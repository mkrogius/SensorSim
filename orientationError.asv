function err = orientationError(sensorHz, accRMS, gyroRMS)
% Params:
%   sensorHz: number of measurements per second
%   accRMS: standard deviation of acceleration measurements (m/s^2)
%   gyroRMS: standard deviation of gyro measurements (rad/s)
% Return value
%   err: The average error after each revolution

% The simulated imu makes
numturns = 1000;
% turns around the x axis



tfinal = 1;
t = 0:1/sensorHz:tfinal;
t = transpose(t);
rotationX = zeros(sensorHz+1,1);

% Calculate the orientation of the imu as a function of time
% assuming angular velocity increases linearly from t=0 til t=0.5
% and decreases linearly from t=0.5 til t=1.0
for i = 1:sensorHz/2+1
    rotationX(i) = 4*pi*t(i).^2;
end
for i = sensorHz/2+1:sensorHz+1
    rotationX(i) = 8*pi*(t(i)-0.5*t(i).^2) - 2*pi;
end

% plot(t,rotationX)
accZ = 9.8*cos(rotationX);
accY = 9.8*sin(rotationX);
gyroX = 8*pi*(0.5-abs(t-0.5));
%figure, plot(t,accZ, t, accY, t, gyroX);

% Ill add magnetometer later

% Add noise
noise = randn(numturns*length(accZ),6);

% Now we have our sensor data
% Lets run Madgwick on it and see how it does

beta = sqrt(3/4)*gyroRMS; % From paper this is the proper gain
AHRS = MadgwickAHRS('SamplePeriod', 1/sensorHz, 'Beta', beta);


quaternion = zeros(numturns*(length(t)-1)+1, 4);
error = zeros(numturns, 4);
time = transpose(0:1/sensorHz:numturns*tfinal);

for j = 1:numturns
    for i = 1:length(t)-1
        curNoise = noise((j-1)*(length(t)-1)+i,:);
        AHRS.UpdateIMU([gyroX(i)+gyroRMS*curNoise(1) ...
            ,gyroRMS*curNoise(2) ...
            ,gyroRMS*curNoise(3)] ...
            , [(accRMS+0.1)*curNoise(4) ...%Extra acc noise b/c of movement
            ,accY(i)+(accRMS+0.1)*curNoise(5) ...
            ,accZ(i)+(accRMS+0.1)*curNoise(6)]);
        quaternion((j-1)*(length(t)-1)+i, :) = AHRS.Quaternion;
    end
    error(j,:) = AHRS.Quaternion;
end
AHRS.UpdateIMU([gyroX(length(t)),0,0], [0,accY(length(t)),accZ(length(t))]);
quaternion(length(time), :) = AHRS.Quaternion;

% Plot as euler angles
euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.

figure(3);
hold on;
plot(time, euler(:,1), 'r');
plot(time, euler(:,2), 'g');
plot(time, euler(:,3), 'b');
title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;

eulererror = quatern2euler(quaternConj(error)) * (180/pi);
s = 1:numturns;
figure(4);
hold on;
plot(s, eulererror(:,1), 'r');
plot(s, eulererror(:,2), 'g');
plot(s, eulererror(:,3), 'b');
title('Euler error');
xlabel('Time');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;

%err = sum(eulererror(:,1))./length(eulererror);
end