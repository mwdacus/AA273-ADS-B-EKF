# AA273-ADS-B-EKF
The ability to estimate aircraft position and velocity
is vital for safe and effective air traffic operations and
management. With the emergence of the Automatic Dependent
Surveillance-Broadcast (ADS-B) network, aircraft are now capable
of sending GNSS positioning and operational data to ground
stations to be used by regional air traffic controllers. On rare (but
possible) situations, aircraft may lose ADS-B signal reception due
to a variety of reasons, so it is important to formally derive a
position solution of the aircraft during ADS-B reception loss.
This paper seeks to use historical crowdsourced ADS-B data
to implement variations of the Extended Kalman Filter (EKF)
during intermittent loss of ADS-B reception. Crowdsourced data
is retrieved from OpenSky Network, a client-based application
that receives air traffic data from a network of ADS-B receivers
around the globe. This newly estimated state is then compared
to a real world flight test case scenario from Edwards Air Force
Base (AFB) in September 2019, where several test flights from a
C-12J Huron were conducted. By comparing this estimated state
with the onboard receiver data, results have indicated that using
ADS-B only does not perform as well as ADS-B with control input
information from UHARS, but manages to track the aircraft for
prolonged trajectories.


