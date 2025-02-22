*option nlp=SCIP;
option nlp=IPOPT;
*option nlp=CONOPT;

parameter
         model_status,
         solver_status;
sets
         i 'vehicles',
         k 'prediction steps';

parameter
         h 'length of the time interval',
         tau 'tau in the car-following model',
         d_0 'd_0 in the car-following model',
         k_dynamics(k) 'dynamic prediction steps used to calculate the differences',
         k_critical_points(k) 'the position where there should be critical points',
         s_init(i) 'initial position of vehicle i',
         s_critical(i,k) 'critical position points of vehicle i at time k',
         v_init(i) 'initial speed of vehicle i',
         veh_dynamics(i) 'indicator of vehicles to calculate the car-following distances';
*$gdxin _gams_py_gdb1
$if not set gdxincname $abort 'no include file name for data file provided'
$gdxin %gdxincname%
$load i k h tau d_0 k_dynamics k_critical_points s_init s_critical v_init veh_dynamics
$gdxin

variable
         f 'cost function',
         f_inst(i,k) 'the instantanuous fuel consumption rate',
         f_position(i,k) 'position penalty',
         a(i,k) 'acceleration of vehicle i at time step k',
         v(i,k) 'speed of vehicle i on lane k at time step k',
         s(i,k) 'position of vehicle i on lane k at time step k';

equations
         cost 'cost function',
         fuel_consumption(i,k) 'calculate fuel consumption rate',
         dynamics_speed(i,k) 'dynamics of the speed',
         dynamics_position(i,k) 'lower bound of a vehicle position',
         car_following(i,k) 'car-following model',
         position_penalty(i,k) 'car-following model';

*Cost function. A relaxed version of Equation (24). We do not require exact match for the critical points. Instead, we try to minimizze the errors for matching those points.
cost..                                                                           f =e= sum(k, sum(i, f_inst(i,k))) + sum(k$(k_critical_points(k) ne 0), sum(i, power(s(i,k) - s_critical(i,k), 2)));

*Equation (25a)
dynamics_speed(i,k)$(k_dynamics(k) ne 0)..                                       v(i,k) - v(i,k-1) =e= h*a(i,k-1);

*Equation (25b)
dynamics_position(i,k)$(k_dynamics(k) ne 0)..                                    s(i,k) - s(i,k-1) =e= h*v(i,k-1) + a(i,k-1)*h*h/2;

*A relaxed version of Equation (25c)
car_following(i,k)$(veh_dynamics(i) ne 0 and k_dynamics(k) ne 0)..               s(i,k) - s(i+1,k) =g= tau*v(i,k);

*Fuel consumption.
fuel_consumption(i,k)..                                                          f_inst(i,k) =e= 0.2736 + 0.0599*v(i,k) + 0.3547*a(i,k) - 0.0058*power(v(i,k),2) + 0.0179*v(i,k)*a(i,k) + 0.0663*power(a(i,k),2) + 0.0002*power(v(i,k),3) + 0.002*power(v(i,k),2)*a(i,k) + 0.0245*v(i,k)*power(a(i,k),2) - 0.0489*power(a(i,k),3);

*Equation (25e)
a.up(i,k) = 4;
a.lo(i,k) = -5;
v.up(i,k) = 11;
v.lo(i,k) = -1;
s.up(i,k) = 700;
s.lo(i,k) = -200;
v.fx(i,'1') = v_init(i);
s.fx(i,'1') = s_init(i);

model m /cost, dynamics_speed, dynamics_position, car_following, fuel_consumption/;

solve m using nlp minimizing f;
model_status = m.Modelstat;
solver_status = m.solvestat;
