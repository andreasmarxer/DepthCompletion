function [class, confidence, left_x, top_y, width, height] = importBoundingBoxes(filename, dataLines)
%IMPORTFILE Import data from a text file
%  [CLASS, CONFIDENCE, LEFT_X, TOP_Y, WIDTH, HEIGHT] =
%  IMPORTFILE(FILENAME) reads data from text file FILENAME for the
%  default selection.  Returns the data as column vectors.
%
%  [CLASS, CONFIDENCE, LEFT_X, TOP_Y, WIDTH, HEIGHT] = IMPORTFILE(FILE,
%  DATALINES) reads data for the specified row interval(s) of text file
%  FILENAME. Specify DATALINES as a positive scalar integer or a N-by-2
%  array of positive scalar integers for dis-contiguous row intervals.
%
%  Example:
%  [class, confidence, left_x, top_y, width, height] = importfile("/home/andreas/Documents/darknet/output-predictions/test.txt", [1, Inf]);
%
%  See also READTABLE.
%
% Auto-generated by MATLAB on 23-Oct-2019 13:28:22

%% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [1, Inf];
end

%% Setup the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 10);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["class", "confidence", "Var3", "left_x", "Var5", "top_y", "Var7", "width", "Var9", "height"];
opts.SelectedVariableNames = ["class", "confidence", "left_x", "top_y", "width", "height"];
opts.VariableTypes = ["categorical", "double", "char", "double", "char", "double", "char", "double", "char", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Specify variable properties
opts = setvaropts(opts, ["Var3", "Var5", "Var7", "Var9"], "WhitespaceRule", "preserve");
opts = setvaropts(opts, ["class", "Var3", "Var5", "Var7", "Var9"], "EmptyFieldRule", "auto");

% Import the data
tbl = readtable(filename, opts);

%% Convert to output type
class = tbl.class;
confidence = tbl.confidence;
left_x = tbl.left_x;
top_y = tbl.top_y;
width = tbl.width;
height = tbl.height;
end