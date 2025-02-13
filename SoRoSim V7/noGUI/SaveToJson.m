% Save a link as a Json file
function SaveToJson(Link, filename)
    Link.PlotFn = func2str(Link.PlotFn);
    Link.r{1} = func2str(Link.r{1});
    % Convert the struct to a JSON string
    jsonStr = jsonencode(Link);
    
    % Format JSON for better readability (optional)
    jsonStr = strrep(jsonStr, '],', sprintf('],\n'));  % Add new lines after commas
    
    % Save JSON string to a file
    fid = fopen(filename, 'w'); % Open file for writing
    if fid == -1
        error('Cannot open file for writing.');
    end
    fprintf(fid, '%s\n', jsonStr);
    fclose(fid);
    
    disp('JSON file created successfully!');
end
