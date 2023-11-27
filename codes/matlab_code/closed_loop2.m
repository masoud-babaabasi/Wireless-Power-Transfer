pskModulator = comm.PSKModulator(M,phOffset,'SymbolMapping',symMap);clear
INPHASE = 1 ;
OFF = 0 ;
ANTIPHASE = 15;
bluetooth_Name = "reciever coil";
load('calibrate_currents_vs_voltage_noload_avg.mat','It')
load('calibrate_currents_vs_voltage_noload_avg.mat','voltage')
s = serialport('COM5' ,115200);
Vt = 5;
STARTCONVERSIONCMD = zeros(1,2);
STARTCONVERSIONCMD(1) = double('S');
STARTCONVERSIONCMD(2) = double('C');
%-----------neighbours-------------------
neighbours = zeros(7,7);
neighbours(1,[2 4 7]) = [1 1 1];
neighbours(2,[1 3 4]) = [1 1 1];
neighbours(3,[2 4 5]) = [1 1 1];
neighbours(4,:) = ones(1,7);
neighbours(4,4) = 0;
neighbours(5,[3 4 6]) = [1 1 1];
neighbours(6,[4 5 7]) = [1 1 1];
neighbours(7,[1 4 6]) = [1 1 1];
%---------------frequency setup---------------
f_single = 841;
f_dual_in = 857;
f_dual_an = 825;
f_cpu = 180 ; % MHz
fres = f_single ; % KHz
set_freq(s , f_cpu , fres)
set_voltage(s, Vt);
%%
states = zeros(1,7);
for i= 1:7
    states = set_transmiter(s , i , OFF , states);
    pause(.1)
end
n_avg =5;
i_avg = 0;
dev = zeros(1,7);
for transmiter_num = 1:7
    set_transmiter(s , transmiter_num , INPHASE);
%     pause(0.1);
    for ii=1:n_avg
        write(s , STARTCONVERSIONCMD , "uint8");
        read_data = read(s,14,"uint8");
        read_data_16 = read_data(1:2:13) + read_data(2:2:14)*256;
        i_avg = i_avg + read_data_16(transmiter_num);
    end
    dev(transmiter_num) = (It(transmiter_num, Vt==voltage ) - i_avg/n_avg)/It(transmiter_num, Vt==voltage );
    set_transmiter(s , transmiter_num , OFF);
    i_avg = 0;
end
[m , transmiter_num] = max(dev);
states = set_transmiter(s , transmiter_num , INPHASE , states);
blutoth_live = 0;
%% check for receiver
set_voltage(s, 8);
time_out = tic;
while(blutoth_live ~= 1)
    t_stop = toc(time_out);
    if t_stop >= 8
        disp("bluetooth Not found. Try again")
        set_voltage(s, 2);
        for i= 1:7
            states = set_transmiter(s , i , OFF , states);
            pause(.1)
        end
        return;
    end
    pause(0.2);
    list = blelist;
    n_list = size(list,1);
    for i = 1:n_list
        if strcmp(bluetooth_Name,list{i,2}) == 1
            blutoth_live = 1;
            break;
        end
    end
    if blutoth_live ~= 1
        Vt = Vt + 0.5 ;
        if Vt > 12 
            Vt = 12;
        end
        set_voltage(s, Vt);
    end
end
% Vt = 5;
% set_voltage(s , Vt);
%%
bluetooth_Name = "reciever coil";
b = ble(bluetooth_Name);
charac = b.Characteristics;
ServUUID = charac(5,2);
CharUUID = charac(5,4);
c = characteristic(b,ServUUID{:,:},CharUUID{:,:});
ble_data = read(c);
ble_data = read(c);
pause(0.5);
clear b c charac
b = ble(bluetooth_Name);
charac = b.Characteristics;
ServUUID = charac(5,2);
CharUUID = charac(5,4);
c = characteristic(b,ServUUID{:,:},CharUUID{:,:});

% s = serialport('COM5' ,115200);
ts =50*1e-3;
Twindow = 5;
t = 0:ts:Twindow;
n = length(t);
data = zeros(n,11);
i = 1;
y_max = 2200;
r = 1;
X = [3/2 , 0 , -3/2 , 0 , -3/2 , 0 , 3/2]*r;
Y = [-sqrt(3)/2 , -sqrt(3) , -sqrt(3)/2 , 0 , sqrt(3)/2 , sqrt(3) , sqrt(3)/2]*r;
Z = [ 1 , 0 , 0 , 0 , 0 , 0 , 0];
x_hex = [ r , r/2 , -r/2 , -r , -r/2 , r/2 , r];
y_hex = [ 0 , sqrt(3)/2 , sqrt(3)/2 , 0 , -sqrt(3)/2 , -sqrt(3)/2 , 0]*r;

figure('Name','Plot values');
scrollsubplot(2,2,1);
scatter3(0,0,0)
hold on
h = scatterbar3(X,Y,Z,r/2);
h = squeeze(h);
h = squeeze(h);
axis equal
zlim([0 2])
title("transmitter currents")
% zlabel("transmitter currents")
for I=1:7
    for J=1:6
        h(I,J).FaceColor = 'B'; 
    end
end
for I =1:7
    plot(x_hex + X(I) , y_hex + Y(I) , 'black','linewidth',3)
end

subplot(2,2,2)
h1(1) = plot(t,data(:,8));
txt(1)=text(0.1,3,['Vr = ',num2str(data(n,8))]);
xlim([0 Twindow]);
xticks(0:1:Twindow)
ylim([0 12]);
ylabel("V receiver",'fontsize',14);
grid on;
yticks(0:2:12);

subplot(2,2,3)
h1(2) = plot(t,data(:,10));
txt(2)=text(0.1,125,['Pr = ',num2str(data(n,10))]);
xlim([0 Twindow]);
xticks(0:1:Twindow)
ylim([0 500]);
ylabel("P receiver( mW )",'fontsize',14);
grid on;
yticks(0:50:500); 

subplot(2,2,4)
h1(3) = plot(t,data(:,11));
txt(3)=text(0.1,2,['V transmiter = ',num2str(data(n,11))]);
xlim([0 Twindow]);
xticks(0:1:Twindow)
ylim([0 12]);
ylabel("V transmiter",'fontsize',14);
grid on;
yticks(0:2:12); 
drawnow;
%%
STARTCONVERSIONCMD = zeros(1,2);
STARTCONVERSIONCMD(1) = double('S');
STARTCONVERSIONCMD(2) = double('C');
read_data = zeros(1,14);
read_data_16 = zeros(1,7);
time_count = 0;
t_start = tic;
BLE_data_in = zeros(1,2);
t_delay = 0.2;%system delay for feedback 
Vref = 8 ;
dead_zone = 0.1 ; 
Kp = 0.03;
pause(0.5)
while(1)
    ble_data = read(c);
%     if length(ble_data) ~= 2
%         clear ble_data;
%         continue;
%     end
%     BLE_data_in =sscanf(string(char(ble_data)),"vin=%f , Iin = %fmA");
    BLE_data_in(1) = ble_data(1) * 14 / 255 + 3 ;
    BLE_data_in(2) = ble_data(2) * 60 / 255 + 10 ;
    write(s , STARTCONVERSIONCMD , "uint8");
    read_data = read(s,14,"uint8");
    read_data_16 = read_data(1:2:13) + read_data(2:2:14)*256;
    data(1:n-1,:) = data(2:n,:);
    data(n,1:7) = read_data_16;
    data(n,8) = BLE_data_in(1);
    data(n,9) = BLE_data_in(2);
    data(n,10) = BLE_data_in(1) * BLE_data_in(2);
    data(n,11) = Vt;
    [m , index_v] = min(abs(Vt-voltage));
    deviation_n = (It(transmiter_num, index_v ) - read_data_16(transmiter_num))/It(transmiter_num, index_v ) ;
%     if deviation_n < 0.6 * dev(transmiter_num)
%         set_transmiter(s , transmiter_num , OFF);
%         for transmiter_num = 1:7
%             states = set_transmiter(s , transmiter_num , ANTIPHASE);
%         %     pause(0.001);
%             write(s , STARTCONVERSIONCMD , "uint8");
%             read_data = read(s,14,"uint8");
%             read_data_16 = read_data(1:2:13) + read_data(2:2:14)*256;
%             dev(transmiter_num) = (It(transmiter_num, index_v ) - read_data_16(transmiter_num))/It(transmiter_num, index_v );
%             states = set_transmiter(s , transmiter_num , OFF);
%         end
%         [m , transmiter_num] = max(dev);
%         states = set_transmiter(s , transmiter_num , ANTIPHASE);
%     end
    for j=1:7
        z = data(n,j)/1000;
       h(j,2).ZData = z.*[0 1 1 0];
       h(j,3).ZData = z.*[0 1 1 0];
       h(j,4).ZData = [z z z z];
       h(j,5).ZData = z.*[0 1 1 0];
       h(j,6).ZData = z.*[0 1 1 0];
       if states(j) == OFF
           bar_color = 'b';
       elseif states(j) == ANTIPHASE 
           bar_color = 'g';
       elseif states(j) == INPHASE
           bar_color = 'r';
       end
        for J=1:6
            h(j,J).FaceColor = bar_color; 
        end
    end
    for j=1:3
        if( j == 1 )
            h1(j).YData = data(:,8);
            t_stop = toc(t_start);
            if t_stop >= t_delay
                time_count = 0;
            end
            vr = data(n,8);
            txt(j).String = ['Vr  = ', num2str(vr)];
            if abs( vr - Vref ) >= dead_zone && time_count == 0
                t_start = tic;
                Vt = Vt - Kp * ( vr - Vref );
                if Vt > 12 
                    Vt = 12 ;
                elseif Vt < 2
                    Vt = 2;
                end
                set_voltage(s , Vt);
                time_count = 1;  
            elseif time_count == 0
                t_start = tic;
            end
        elseif( j == 2 )
            h1(j).YData = data(:,10);
            txt(j).String = ['P receiver  = ', num2str(data(n,10))];
        elseif( j == 3 )
            h1(j).YData = data(:,11);
            txt(j).String = ['V transmiter  = ', num2str(Vt)];
        end        
    end
    if Vt > 4.9 && vr <= 8.1 && fres == f_single
%       transmiter_num = find(states == INPHASE);
      dev = zeros(1,7);
      for i_t = 1:7
          if neighbours(transmiter_num , i_t) == 1
            fres = f_dual_in ; 
            set_freq(s , f_cpu , fres)
            pause(0.1);
            states = set_transmiter(s , i_t , INPHASE , states);
            pause(0.1);
            ble_data = read(c);
            BLE_data_in(1) = ble_data(1) * 14 / 255 + 3 ;
            dev(i_t) = BLE_data_in(1);
            states = set_transmiter(s , i_t , OFF , states);
            fres = f_single ; 
            set_freq(s , f_cpu , fres)
            pause(0.1);
          end
      end 
      [m , transmiter_num] = max(dev);
      fres = f_dual_in ; 
      set_freq(s , f_cpu , fres)
      pause(0.1);
      states = set_transmiter(s , transmiter_num , INPHASE , states);
      continue;
    end
    if Vt > 3.9 && vr <= 8.1 && fres == f_dual_in
%       transmiter_num = find(states == INPHASE);
      dev = zeros(1,7);
      for i_t = 1:7
          if neighbours(transmiter_num , i_t) == 1
            Vt = 4.7;
            set_voltage(s , Vt);
            pause(0.1);
            states = set_transmiter(s , i_t , OFF , states);
            fres = f_single ; 
            set_freq(s , f_cpu , fres)
            pause(0.1);
          end
      end 
    end
end

