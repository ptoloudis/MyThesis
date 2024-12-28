try
    state = load('Copter','copter');
catch
    run('Copter.m')
    fprintf('Could not find Hexsoon.mat file, running copter.m\n')
    return
end

state.port = 9002;

fprintf("Start\n");
jsonStr = jsonencode(state);

% Write JSON string to a file
fileID = fopen('copter/copter.json', 'w');
if fileID == -1
    error('Failed to open the file.');
end
fwrite(fileID, jsonStr, 'char');
fclose(fileID);
disp('JSON file saved successfully!');
