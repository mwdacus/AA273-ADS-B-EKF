%% Code Information
%*************************************************************************
%AA 273                        EKFMain                       Michael Dacus

%Function Description: Implement EKF on nonholonomic mobile robot model

%Inputs:
    %Dynamics of the state model
    %Measurement Model
%Output:
    %EKF Estimated State, Compared to State
   

%**************************************************************************

clear
clc
close all

%% Data Import/Manipulation
[adsbfile_csv,adsbpath_csv] = uigetfile('*.csv');
[gpsfile_csv,gpspath_csv] = uigetfile('*.csv');
adsbdata=readtimetable([adsbpath_csv,adsbfile_csv]);
gpsdata=readtimetable([gpspath_csv,gpsfile_csv]);
adsbdata=sortrows(adsbdata,'mintime','ascend');
gpsdata=sortrows(gpsdata,'mintime','ascend');

%Select Start and End ADSB Data 
startind=find(adsbdata.mintime=='2019-09-11 17:29:08.770');
endind=find(adsbdata.mintime=='2019-09-11 17:36:47.436');
adsbdata=adsbdata(startind:endind,:);

%% Aircraft Segment Split EKF run
reltime=seconds(adsbdata.mintime-adsbdata.mintime(1));
delta_t=diff(reltime);

gapsind={1 find((delta_t>4)) find((delta_t>4))+1 length(reltime)};
gapsind=vertcat(gapsind{:});
gapsind=sort(gapsind,'ascend');
adsbsync=cell(1,length(gapsind)/2);
counter=1;
for i=1:2:length(gapsind)
    gaptime=adsbdata.mintime(gapsind(i):gapsind(i+1));
    adsbseg=adsbdata(gapsind(i):gapsind(i+1),:);
    [~,ind1]=min(abs(adsbdata.mintime(gapsind(i))-gpsdata.mintime));
    [~,ind2]=min(abs(adsbdata.mintime(gapsind(i+1))-gpsdata.mintime));
    adsbsync{counter}=synchronize(adsbseg(:,1:7),gpsdata.mintime(ind1:ind2),...
        'linear');
    counter=counter+1;
end
adsbdata=vertcat(adsbsync{:});

%Filter GPS Data
starttime=adsbdata.mintime(1);
endtime=adsbdata.mintime(end);
[~,I_start]=min(abs(gpsdata.mintime-starttime));
[~,I_end]=min(abs(gpsdata.mintime-endtime));
gpsdata=gpsdata(I_start:I_end,:);

%trajplot(adsbdata,gpsdata)
%% EKF Main Script
%Convert to ENU for EKF
origin=table2array(gpsdata(1,:));
ENU_data_ADSB=[lla2enu([adsbdata.lat,adsbdata.lon,adsbdata.alt], ...
    [origin(1),origin(2),origin(3)],"ellipsoid"), adsbdata.velocity, ...
    adsbdata.heading, adsbdata.vertrate, adsbdata.nic];
ENU_data_GPS=[lla2enu([gpsdata.lat,gpsdata.lon,gpsdata.alt],...
    [origin(1),origin(2),origin(3)],"ellipsoid") gpsdata.velocity,...
    gpsdata.heading gpsdata.vertrate];

%Method 1 Approach
ekfstateenuadsb=EKF_ADSB(ENU_data_ADSB,ENU_data_GPS,reltime);
%Method 2 Approach
ekfstateenugps=EKF_ADSB_GPS(ENU_data_ADSB,ENU_data_GPS,reltime);

%Convert Back to LLA:
ekfstatellaadsb=enu2lla([ekfstateenuadsb(:,1),ekfstateenuadsb(:,2),ekfstateenuadsb(:,3)],...
    [origin(1),origin(2),origin(3)],"ellipsoid");
ekfstatellagps=enu2lla([ekfstateenugps(:,1),ekfstateenugps(:,2),ekfstateenugps(:,3)],...
    [origin(1),origin(2),origin(3)],"ellipsoid");


trajplot(adsbdata,gpsdata,ekfstatellaadsb,ekfstatellagps)
altplot(adsbdata.mintime,adsbdata,gpsdata,ekfstatellaadsb,ekfstatellagps)
errorplot(adsbdata.mintime,ENU_data_GPS(:,1:3),ekfstateenuadsb(:,1:3),ekfstateenugps(:,1:3))
%% Plot Metrics
% Plot 2D Trajectory
function trajplot(adsbdata,gpsdata,ekfadsb,ekfgps)
    gx=geoaxes;
    geoplot(adsbdata.lat,adsbdata.lon,'og','MarkerSize',4,'LineWidth',1)
    hold on
    geoplot(gpsdata.lat,gpsdata.lon,'or','MarkerSize',4,'LineWidth',1)
    geoplot(ekfadsb(:,1),ekfadsb(:,2),'-b')
    geoplot(ekfgps(:,1),ekfgps(:,2),'-.m','LineWidth',1)
    legend('ADSB','GPS','EKF (ADS-B Only)', 'EKF (ADS-B w/ UHARS Input)','Location','northwest')
    geobasemap('topographic')
    hold off
end
%Plot Altitude v. Time
function altplot(time,adsbdata,gpsdata,ekfadsb,ekfgps)
    figure('color','w')
    plot(time,adsbdata.alt,'og','MarkerSize',4,'LineWidth',1)
    hold on
    plot(time,gpsdata.alt,'or','MarkerSize',4,'LineWidth',1)
    plot(time,ekfadsb(:,3),'-b','MarkerSize',4)
    plot(time,ekfgps(:,3),'-.m','MarkerSize',4,'LineWidth',1)
    grid on
    legend('ADSB','GPS','EKF (ADS-B) Only','EKF (ADS-B) w/ UHARS Input')
    xlabel('Time (UTC)')
    ylabel('Altitude (MSL) [m]')
end


function errorplot(time,ENU_gps,ENU_ekf,ENU_ekf_gps)
    figure('color','w')
    err_ENU=ENU_ekf-ENU_gps;
%     err_enu_gps=ENU_ekf_gps-ENU_gps;
    plot(time,err_ENU(:,1))
    hold on
    plot(time,err_ENU(:,2))
    plot(time,err_ENU(:,3))

%     plot(time,err_enu_gps(:,1))
%     plot(time,err_enu_gps(:,2))
%     plot(time,err_enu_gps(:,3),'--r')



    xlabel('Time (UTC)')
    ylabel('Distance Error [m]')
    legend('East-West Error','North-South Error', 'Altitude Error')
    grid on
end