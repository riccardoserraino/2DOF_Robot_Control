figure;
hold on
grid on
box on
plot(xc_real,yc_real);
plot(xc_des,yc_des);

figure;
hold on
grid on
box on
plot(out.xc_sim,out.yc_sim);
plot(xc_des,yc_des);

figure;
error_x = abs(xc_des-xc_real);
subplot(2,1,1);
plot(error);
grid on;
box on;
error_y = abs(yc_des-yc_real);
subplot(2,1,2);
plot(error2);
grid on;
box on;



