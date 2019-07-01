clear all

Array = csvread('PIDdstfinal.csv');
[rown, coln] = size(Array);

time = Array(:,3);
for k = 2:rown
    time(k,1) = time(k-1,1) + time(k,1);
end

roll = Array(:,1);
pitch = Array(:,2);
Ts = Array(:,3);

figure
plot(time,roll)

hold on
plot(time,pitch)
xlim([0 20000])
legend('roll','pitch')
xlabel('time (ms)')
ylabel('Angular states (degrees)')