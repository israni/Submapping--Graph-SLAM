figure;
plot(vert.x(1:1227),vert.y(1:1227));
hold on;
res = [vert.x(1),vert.y(1),vert.th(1)];
for i = 1:length(edge.IDout)
    res(end+1,:) = res(i,:) + [sqrt(edge.dx(i)^2+edge.dy(i)^2).*[cos(res(i,3)+edge.dth(i)),sin(res(i,3)+edge.dth(i))],edge.dth(i)];
    plot(res(i:i+1,1),res(i:i+1,2),'k');
end