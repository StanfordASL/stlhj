%Plot Fixer
%Written by: Matt Svrcek  12/05/2001

%Run this script after generating the raw plots.  It will find
%all open figures and adjust line sizes and text properties.

%Change the following values to suit your preferences.  The variable
%names and comments that follow explain what each does and their options.

plotlsize = 2; %thickness of plotted lines, in points
axislsize = 1.5; %thickness of tick marks and borders, in points
markersize = 8;  %size of line markers, default is 6

%font names below must exactly match your system's font names
%check the list in the figure pull down menu under Tools->Text Properties
%note, the script editor does not have all the fonts, so use the figure menu

axisfont = 'Helvetica'; %changes appearance of axis numbers
axisfontsize = 12;            %in points
axisfontweight = 'normal';    %options are 'light' 'normal' 'demi' 'bold' 
axisfontitalics = 'normal';   %options are 'normal' 'italic' 'oblique'

legendfont = 'Helvetica'; %changes text in the legend
legendfontsize = 14;
legendfontweight = 'normal';
legendfontitalics = 'normal';

labelfont = 'Helvetica';  %changes x, y, and z axis labels
labelfontsize = 14;  
labelfontweight = 'normal'; 
labelfontitalics = 'normal';

titlefont = 'Helvetica';  %changes title
titlefontsize = 14;
titlefontweight = 'normal';
titlefontitalics = 'normal';

textfont = 'Helvetica';   %changes text
textfontsize = 14;
textfontweight = 'normal';
textfontitalics = 'normal';


%stop changing things below this line
%----------------------------------------------------
axesh = findobj('Type', 'axes');
legendh = findobj('Tag', 'legend');
lineh = findobj(axesh, 'Type', 'line');
axestexth = findobj(axesh, 'Type', 'text');

set(lineh, 'LineWidth', plotlsize)
set(lineh, 'MarkerSize', markersize)
set(axesh, 'LineWidth', axislsize)
set(axesh, 'FontName', axisfont)
set(axesh, 'FontSize', axisfontsize)
set(axesh, 'FontWeight', axisfontweight)
set(axesh, 'FontAngle', axisfontitalics)
set(axestexth, 'FontName', textfont)
set(axestexth, 'FontSize', textfontsize)
set(axestexth, 'FontWeight', textfontweight)
set(axesh, 'Box','on')
for i = 1:1:length(axesh)
   %legend(axesh(i))
   set(get(gca,'XLabel'), 'FontName', labelfont)
   set(get(gca,'XLabel'), 'FontSize', labelfontsize)
   set(get(gca,'XLabel'), 'FontWeight', labelfontweight)
   set(get(gca,'XLabel'), 'FontAngle', labelfontitalics)
   set(get(gca,'YLabel'), 'FontName', labelfont)
   set(get(gca,'YLabel'), 'FontSize', labelfontsize)
   set(get(gca,'YLabel'), 'FontWeight', labelfontweight)
   set(get(gca,'YLabel'), 'FontAngle', labelfontitalics)
   set(get(gca,'ZLabel'), 'FontName', labelfont)
   set(get(gca,'ZLabel'), 'FontSize', labelfontsize)
   set(get(gca,'ZLabel'), 'FontWeight', labelfontweight)
   set(get(gca,'ZLabel'), 'FontAngle', labelfontitalics)
   set(get(gca,'Title'), 'FontName', titlefont)
   set(get(gca,'Title'), 'FontSize', titlefontsize)
   set(get(gca,'Title'), 'FontWeight', titlefontweight)
   set(get(gca,'Title'), 'FontAngle', titlefontitalics)
end
