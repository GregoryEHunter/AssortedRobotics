r = .445;
theta = -30;
x = r*sind(theta);
y = r*cosd(theta);
m = -x/y;
b = y-m*x;

x2 = linspace(-4,4,1000);
y2 = m*x2+b;



r = .445;
theta = -320;
x = r*sind(theta);
y = r*cosd(theta);
m = -x/y;
b = y-m*x;

x3 = linspace(-4,4,1000);
y3 = m*x2+b;

plot(x2,y2,x3,y3)
ylim([-4,4])
legend
hold on
plot(0,0,'r*')