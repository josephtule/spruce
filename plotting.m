clear
opts = delimitedTextImportOptions("NumVariables", 3);

directory = pwd;  % Specify the directory path
files = dir(fullfile(directory, 'output*'));  % Get a list of files matching the pattern

num_sats = numel(files);  % Count the number of files

% Specify range and delimiter
opts.DataLines = [2, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["x", "y", "z"];
opts.VariableTypes = ["double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data and plot
figure(1)
hold on
grid on


xlabel("x"),ylabel("y"),zlabel("z")
axis equal
view(-37.5,30)
leg = cell(1,num_sats);
xout = []; yout = []; zout = [];
for i = 0:num_sats-1
    fn = "output" + num2str(i);
    filename = fn + ".txt";
    output.(fn) = readtable(filename,opts);
    satname = "sat" + num2str(i);
    x.(satname) = output.(fn).x;
    y.(satname) = output.(fn).y;
    z.(satname) = output.(fn).z;
    % xout = [xout x.(satname)];
    % yout = [yout y.(satname)];
    % zout = [zout z.(satname)];
    plot3(x.(satname),y.(satname),z.(satname))

    leg{i+1} = satname;
end
lims = [-1 1] * 400000000;
xlim(lims)
ylim(lims)
zlim(lims)



% for j = 1:length(x.(satname))
% 
%         plot3(xout(1:j,:),yout(1:j,:),zout(1:j,:))
%         xlim([-10000e3,10000e3])
%         ylim([-10000e3,10000e3])
%         zlim([-10000e3,10000e3])
% 
%     drawnow
% end
legend(leg,'location','best')
clear opts output

