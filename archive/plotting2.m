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

for i = 0:num_sats-1
    % Import the data
    fn = "output" + num2str(i);
    filename = fn + ".txt";
    output.(fn) = readtable(filename, opts);
    satname = "sat" + num2str(i);
    x.(satname) = output.(fn).x;
    y.(satname) = output.(fn).y;
    z.(satname) = output.(fn).z;
    
    % Plot satellite orbit in individual figure
    figure(i+1)
    plot3(x.(satname),y.(satname),z.(satname))
    title(satname)
    xlabel("x"), ylabel("y"), zlabel("z")
    axis equal
    grid on
    lims_sat = [min(x.(satname)) max(x.(satname)); min(y.(satname)) max(y.(satname)); min(z.(satname)) max(z.(satname))];
    xlim([min(lims_sat(:,1)), max(lims_sat(:,2))])
    ylim([min(lims_sat(:,1)), max(lims_sat(:,2))])
    zlim([min(lims_sat(:,1)), max(lims_sat(:,2))])
end

% Plot all orbits in the same figure (N+1)
figure(num_sats+1)
hold on
grid on
xlabel("x"), ylabel("y"), zlabel("z")
axis equal
leg = cell(1,num_sats);

for i = 0:num_sats-1
    satname = "sat" + num2str(i);
    plot3(x.(satname),y.(satname),z.(satname))
    leg{i+1} = satname;
end

lims = [-1 1] * 400000000;
xlim(lims)
ylim(lims)
zlim(lims)
legend(leg,'location','best')

clear opts output
