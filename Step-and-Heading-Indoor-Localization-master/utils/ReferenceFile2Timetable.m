function references = ReferenceFile2Timetable(filename, dataLines)
%IMPORTFILE Import data from a text file
%  REFERENCES = IMPORTFILE(FILENAME) reads data from text file FILENAME
%  for the default selection.  Returns the data as a table.
%
%  REFERENCES = IMPORTFILE(FILE, DATALINES) reads data for the specified
%  row interval(s) of text file FILENAME. Specify DATALINES as a
%  positive scalar integer or a N-by-2 array of positive scalar integers
%  for dis-contiguous row intervals.
%
%  Example:
%  references = importfile("/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/SHS Code/datasets/lopen1/references.txt", [6, Inf]);
%
%  See also READTABLE.
%
% Auto-generated by MATLAB on 27-Aug-2020 12:28:15

%% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [7, Inf];
end

%% Setup the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 3);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = [" ", "-"];

% Specify column names and types
opts.VariableNames = ["elapsed", "Var2", "Var3"];
opts.SelectedVariableNames = "elapsed";
opts.VariableTypes = ["double", "string", "string"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";
opts.ConsecutiveDelimitersRule = "join";

% Specify variable properties
opts = setvaropts(opts, ["Var2", "Var3"], "WhitespaceRule", "preserve");
opts = setvaropts(opts, ["Var2", "Var3"], "EmptyFieldRule", "auto");

% Import the data
if exist(filename, 'file') == 2
    
    references = readtable(filename, opts);
    references.elapsed = seconds(references.elapsed);
    references = table2timetable(references);
    references.door_detection = ones(height(references),1);
else
    disp(['door handle reference file: ' filename ' cannot be found'])
    references = [];
end

end