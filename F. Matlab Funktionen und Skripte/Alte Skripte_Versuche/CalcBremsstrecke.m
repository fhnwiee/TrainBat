v_data = zeros(1000,1);
v = 75;
v_data(1) = v;

a_max = -1.1;

a0 = zeros(1000,1);
a0(1) = 0;

dt = 0.1;

da = -0.1;

x = zeros(1000,1);

i = 2;

while (v > 0)
    
    x(i) = x(i) + v/3.6 * dt;
    if (a0(i-1) <= a_max)
        a0(i) = a_max;
    elseif (a_max < a0(i-1))
        a0(i) = a0(i-1) + da;
    end
    v = (v/3.6 + a0(i)*dt)*3.6;
    v_data(i) = v;
    i = i + 1;

end


x_const = abs((75/3.6)^2/(2 * (-1.1)));

for i = 1:length(x)
    x(i,2) = sum(x(1:i,1));
end

time = 0:0.1:99.9;

plot(time,x)
plot(time,a0)
plot(time,v_data)

a = -1.3;
v = zeros(100,1);
v(1) = 75;
x  = zeros(100,1);

%% Runterbeschleunigen

a0 = 1.3;
v0 = 75;
dt = 0.1;

for i = (0:0.01:abs(a0))
    
    v0 = ((v0)/3.6 - i * dt)*3.6;

end


%% gesamte Bremsstrecke berechnen:

v0 = 75.4812;
a0 = 0;
dt = 0.1;
x_break = 0;
v1 = 0;



for i = a0:-0.01:FZ.a_break_max
    v0 = (v0/3.6 + i * dt)*3.6;
    x_break = x_break + (v0/3.6) * dt;
end

for i = a0:0.01:abs(FZ.a_break_max)
    v1 = (v1/3.6 + i * dt) * 3.6;
    x_break = x_break + (v1/3.6) * dt;
end

x_break = x_break + (v0/3.6)^2/(2 * abs(FZ.a_break_max)) - (v1/3.6)^2/(2 * abs(FZ.a_break_max));

































