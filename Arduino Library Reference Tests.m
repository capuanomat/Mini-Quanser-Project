% Start by creating arduino variable, this allows you to switch ports on
% and off
a = arduino('com7', 'uno');
writeDigitalPin(a, 'D11', 1);
writeDigitalPin(a, 'A3', 1);
writeDigitalPin(a, 'D11', 0);
writeDigitalPin(a, 'A3', 0);

% MATTHIEU INCLUDE STUFF ABOUT MOTORS (LED FOR NOW) HERE
if (Roll > 90)
    writeDigitalPin(s, 'A3', 1); %Turn on blue light
elseif (Roll < 90)
    writeDigitalPin(s, 'D11', 1);
    %Turn on red light
end