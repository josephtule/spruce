clear
addpath("outputs/bin")

plotsphere = true;

params.rad = 6378137/1000;
directory = fullfile(pwd, 'outputs/bin');
files = dir(fullfile(directory, 'output_sat*'));
num_sats = numel(files);

files_other = dir(fullfile(directory, 'output_other_*'));
num_other = numel(files_other);

allX = [];
allY = [];
allZ = [];

% generate sphere
[xplanet, yplanet, zplanet] = sphere(100);

for i = 1:num_sats
    fn = "output_sat" + num2str(i);
    filename = fullfile(directory, fn + ".bin");
    fid = fopen(filename, 'rb');
    
    % read binary data (assuming doubles for x, y, z)
    data = fread(fid, [3, inf], 'double'); 
    fclose(fid);

    satname = "sat" + num2str(i);
    x.(satname) = data(1, :).';
    y.(satname) = data(2, :).';
    z.(satname) = data(3, :).';

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
    hold on

    if plotsphere
        % plot sphere
        hSurface = surf(xplanet*params.rad, yplanet*params.rad, zplanet*params.rad);
        set(hSurface, 'FaceColor', [1 1 2]/2, ...
            'FaceAlpha', 1, 'EdgeColor', 'none', 'FaceLighting', 'gouraud')
        camlight
    end
    xlim([centroid(1)-max_dist, centroid(1)+max_dist])
    ylim([centroid(2)-max_dist, centroid(2)+max_dist])
    zlim([centroid(3)-max_dist, centroid(3)+max_dist])
    hold off
end

% Plot all orbits in the same figure (N+1)
figure(num_sats + 1)
hold on
grid on
xlabel("x"), ylabel("y"), zlabel("z")
axis square
leg = cell(1, num_sats);

for i = 1:num_sats
    satname = "sat" + num2str(i);
    plot3(x.(satname), y.(satname), z.(satname))
    leg{i} = satname;
end
k = i;

% Reading and plotting other bodies
for i = 1:num_other
    fn = replace(files_other(i).name, ".bin", "");
    filename = fullfile(directory, fn + ".bin");
    fid = fopen(filename, 'rb');

    % read binary data (assuming doubles for x, y, z)
    data = fread(fid, [3, inf], 'double'); 
    fclose(fid);

    othername = "other" + num2str(i);
    x.(othername) = data(1, :).';
    y.(othername) = data(2, :).';
    z.(othername) = data(3, :).';
    plot3(x.(othername), y.(othername), z.(othername))
    leg{k + i} = othername;
end

lims = [-1 1] * max([max(abs(allX)), max(abs(allY)), max(abs(allZ))]) * 1.2;  
xlim(lims)
ylim(lims)
zlim(lims)

if plotsphere
    % plot sphere
    hSurface = surf(xplanet*params.rad, yplanet*params.rad, zplanet*params.rad);
    set(hSurface, 'FaceColor', [1 1 2]/2, ...
        'FaceAlpha', 1, 'EdgeColor', 'none', 'FaceLighting', 'gouraud')
    camlight
    leg{k + i + 1} = "central body";
end
hold off
title("Orbit System")
legend(leg, 'location', 'best')
