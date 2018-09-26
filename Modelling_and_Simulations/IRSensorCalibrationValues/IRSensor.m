clear a
a = arduino();

n = 500;
voltage = zeros(n,1);

while n > 0
    voltage(n) = readVoltage(a,'A0');
    pause(0.01);
    n = n - 1;
end

