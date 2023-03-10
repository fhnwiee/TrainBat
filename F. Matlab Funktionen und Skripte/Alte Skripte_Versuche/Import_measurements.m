%% Import data from text file
% Script for importing data from the following text file:
%
%    filename: C:\Users\Diego\switchdrive\Master of Science\P9 Master Thesis\05_Modellierung Zugsystem\Datenexport_NISV_BlattC_ZHUW_SP04_20210111.csv
%
% Auto-generated by MATLAB on 20-May-2022 16:47:02

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 25);

% Specify range and delimiter
opts.DataLines = [2, Inf];
opts.Delimiter = ";";

% Specify column names and types
opts.VariableNames = ["MIV", "DatumZeit", "Zeit", "DiDok", "Werk", "Ort", "Schaltfeld", "altTisID", "Richtung", "Linie", "km_von", "km_bis", "U8minMittelkV", "I8minMittelA", "P8minMittelMW", "StatusMessungU", "StatusMessungI", "StatusMessungP", "AnzahlBeruecksichtigteMessungenU", "AnzahlNichtBeruecksichtigteMessungenU", "AnzahlBeruecksichtigteMessungenI", "AnzahlNichtBeruecksichtigteMessungenI", "AnzahlBeruecksichtigteMessungenP", "AnzahlNichtBeruecksichtigteMessungenP", "Bemerkungen"];
opts.VariableTypes = ["double", "datetime", "datetime", "string", "string", "string", "double", "double", "string", "double", "double", "double", "double", "double", "double", "categorical", "categorical", "categorical", "double", "double", "double", "double", "double", "double", "string"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Specify variable properties
opts = setvaropts(opts, ["DiDok", "Werk", "Ort", "Richtung", "Bemerkungen"], "WhitespaceRule", "preserve");
opts = setvaropts(opts, ["DiDok", "Werk", "Ort", "Richtung", "StatusMessungU", "StatusMessungI", "StatusMessungP", "Bemerkungen"], "EmptyFieldRule", "auto");
opts = setvaropts(opts, "DatumZeit", "InputFormat", "dd.MM.yyyy HH:mm");
opts = setvaropts(opts, "Zeit", "InputFormat", "HH:mm:ss");
opts = setvaropts(opts, "Schaltfeld", "TrimNonNumeric", true);
opts = setvaropts(opts, "Schaltfeld", "ThousandsSeparator", ",");

% Import the data
Meas = readtable("C:\Users\Diego\switchdrive\Master of Science\P9 Master Thesis\05_Modellierung Zugsystem\Datenexport_NISV_BlattC_ZHUW_SP04_20210111.csv", opts);
MeasMonday = Meas(1:289,:);
MeasSunday = Meas(1730:2017,:);

%% Clear temporary variables
clear opts