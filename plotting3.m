clear
addpath("outputs")
opts = delimitedTextImportOptions("NumVariables", 3);

directory = fullfile(pwd, 'outputs');  % Modify the directory path to include "outputs"
files = dir(fullfile(directory, 'output_sat*'));  % Get a list of files matching the pattern

num_sats = numel(files);  % Count the number of files

% Specify range and delimiter
opts.DataLines = [1, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["x", "y", "z"];
opts.VariableTypes = ["double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

allX = [];
allY = [];
allZ = [];

for i = 1:num_sats
    % Import the data
    fn = "output_sat" + num2str(i);
    filename = fullfile(directory, fn + ".txt");
    output.(fn) = readtable(filename, opts);
    satname = "sat" + num2str(i);
    x.(satname) = output.(fn).x;
    y.(satname) = output.(fn).y;
    z.(satname) = output.(fn).z;
    
    % Store all coordinates for final limits
    allX = [allX; x.(satname)];
    allY = [allY; y.(satname)];
    allZ = [allZ; z.(satname)];

    % Compute the centroid of the orbit
    centroid = [mean(x.(satname)), mean(y.(satname)), mean(z.(satname))];
    
    % Determine the largest distance from the centroid for setting axis limits
    dist_from_centroid = sqrt((x.(satname)-centroid(1)).^2 + (y.(satname)-centroid(2)).^2 + (z.(satname)-centroid(3)).^2);
    max_dist = max(dist_from_centroid);
    
    % Plot satellite orbit in individual figure centered at the centroid
    figure(i)
    plot3(x.(satname),y.(satname),z.(satname))
    title(satname)
    xlabel("x"), ylabel("y"), zlabel("z")
    axis square
    grid on
    xlim([centroid(1)-max_dist, centroid(1)+max_dist])
    ylim([centroid(2)-max_dist, centroid(2)+max_dist])
    zlim([centroid(3)-max_dist, centroid(3)+max_dist])
end

% Plot all orbits in the same figure (N+1)
figure(num_sats+1)
hold on
grid on
xlabel("x"), ylabel("y"), zlabel("z")
axis square
leg = cell(1,num_sats);

for i = 1:num_sats
    satname = "sat" + num2str(i);
    plot3(x.(satname),y.(satname),z.(satname))
    leg{i} = satname;
end

lims = [-1 1] * max([max(abs(allX)), max(abs(allY)), max(abs(allZ))]) * 1.2;  % Find the maximum boundary dynamically
xlim(lims)
ylim(lims)
zlim(lims)
legend(leg,'location','best')

clear opts output
