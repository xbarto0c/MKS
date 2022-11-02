clear all;
close;

%% CSV Read
table = readtable("ntc.csv");
array = table2array(table);

temperature = array(:,1);
resistance = 1e3 .* array(:,2);
R1_divider_resistance = 10e3;

ad_values = 2^10 * (resistance ./ (R1_divider_resistance + resistance)); % výpočet hodnot AD převodníku

polynom = polyfit(ad_values, temperature, 10); % výpočet sady dat, s 2^10 hodnotami
ad_values_interp = 0:1023;
temperature_interp = round(polyval(polynom, ad_values_interp), 1);

writematrix(temperature_interp*10, 'data.txt');

figure(1);
plot(ad_values, temperature, 'go');
hold on;
plot(ad_values_interp, temperature_interp);
title("Závislost teploty NTC termistoru na jeho odporu");
xlabel("{\it ADC} [-]");
ylabel("Teplota [°C]");