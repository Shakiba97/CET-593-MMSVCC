option minlp=SCIP;
parameter
         model_status,
         solver_status;
sets
         i 'vehicles',
         j 'lanes',
         l 'signal phases',
         k 'prediction steps',
         vehicle_indi 'indicators showing that whether there is a vehicle i at lane j',
         m 'crossings';

parameter
         h 'length of the time interval, i.e., the step size',
         d_0 'd_0 in the car-following model',
         k_dynamics(k) 'dynamic prediction steps used to calculate the differences',
         lane_length(j) 'length of lane j',
         veh_dynamics(i,j) 'indicator of vehicles to calculate the car-following distances',
         gamma(k) 'discount ratio',
         s_init(i,j) 'initial position of vehicle i at lane j',
         wt_init(i,j) 'initial waiting time of vehicle i at lane j',
         p_init(l) 'initial phases',
         ped_Demand(m,k) 'Pedestrian demand for each crossing m for the next k steps of prediction horizon';

*$gdxin _gams_py_gdb0
$if not set gdxincname $abort 'no include file name for data file provided'
$gdxin %gdxincname%
$load i j l k vehicle_indi m h d_0 k_dynamics lane_length veh_dynamics gamma s_init wt_init p_init ped_Demand
$gdxin

variable
         f 'cost function',
         f_throughput 'throughput cost',
         f_dist 'distance cost',
         f_transit 'phase transition cost',
         f_delay 'delay cost',
         f_ped_throughput 'pedestrian throughput cost',
         v(i,j,k) 'speed of vehicle i on lane j at time step k',
         s(i,j,k) 'position of vehicle i on lane j at time step k';

binary variable
         g(i,j,k) 'whether vehicle i at lane j at time r passed the interseciton',
         p(l,k) 'whether phase l at time step k is green',
         p_c(l,k) 'whether phase l changed at time step k (e.g., from 0 to 1 or from 1 to 0)',
         r(j,k) 'whether lane j has the right of way',
         q(m,k) 'whether crossing m has the right of way';

equations
         cost 'cost function',
         cost_throughput 'throughput cost function',
         cost_distance 'traveled distance cost function',
         cost_transition 'lphase transition cost function',
         cost_delay 'delay cost function',
         cost_ped_throughput 'pedestrian throughput cost',
         vehicle_dynamics_1(i,j,k) 'vehicle dynamics',
         vehicle_dynamics_2(i,j,k) 'vehicle dynamics',
         car_following(i,j,k) 'car-following model',
         vehicle_position_1(i,j,k) 'vehicle position indication',
         vehicle_position_2(i,j,k) 'vehicle position indication',
         traffic_rule(i,j,k) 'traffic rule',
         signal_rule_1(l,k) 'signal rule',
         signal_rule_2(k) 'signal rule',
         signal_rule_3(k) 'signal rule',
         signal_rule_4 'pedestrian constraint'
         phase_equal_1(k) 'signal phase constraint',
         phase_equal_2(k) 'signal phase constraint',
         phase_equal_3(k) 'signal phase constraint',
         phase_equal_4(k) 'signal phase constraint',
         phase_equal_5(k) 'signal phase constraint',
         phase_equal_6(k) 'signal phase constraint',
         phase_equal_7(k) 'signal phase constraint',
         phase_equal_8(k) 'signal phase constraint',
         phase_equal_9(k) 'signal phase constraint',
         phase_equal_10(k) 'signal phase constraint',
         phase_equal_11(k) 'signal phase constraint',
         phase_equal_12(k) 'signal phase constraint',
         ped_phase_equal_1 'pedestrian phase constraint',
         ped_phase_equal_2 'pedestrian phase constraint',
         ped_phase_equal_3 'pedestrian phase constraint',
         ped_phase_equal_4 'pedestrian phase constraint';

$ontext
The cost function. Should be Equation (22). However, I found it hard for GAMS to converge using Equation (22) only. Therefore, I added some extra cost functions
to stablize the GAMS solving process. Equation (22) is essentially cost_throughput. cost_distance, cost_transition, and cost_delay are extra cost functions.
The weights are selected by trial and error.

I guess g here is not same as in the paper. 1 if passed, 0 if did not pass.

you can add pedestrian delay too if this alone did not work
$offtext
cost..                                                                           f =e= 0.8*[f_throughput + f_dist/100 + f_transit*5 + f_delay/50] + 0.2*[10*f_ped_throughput];
cost_throughput..                                                                f_throughput =e= sum(vehicle_indi(i,j), sum(k, (1-g(i,j,k))));
cost_distance..                                                                  f_dist =e= -sum(vehicle_indi(i,j), sum(k, (s(i,j,k) - s_init(i,j))*gamma(k))) / sum(i, sum(j, vehicle_indi(i,j)));
cost_transition..                                                                f_transit =e= sum(l, sum(k, p_c(l,k)));
cost_delay..                                                                     f_delay =e= sum(vehicle_indi(i,j),  sum(k, wt_init(i,j)*(1-g(i,j,'7'))*gamma(k))) / (sum(i, sum(j, vehicle_indi(i,j))));
cost_ped_throughput..                                                            f_ped_throughput =e= -sum(k, sum(m, q(m,k)*ped_Demand(m,k)));

*A relaxed version of Equation (23a).
vehicle_dynamics_1(i,j,k)$(vehicle_indi(i,j) and k_dynamics(k) ne 0)..           s(i,j,k) - s(i,j,k-1) =g= 0;
vehicle_dynamics_2(i,j,k)$(vehicle_indi(i,j) and k_dynamics(k) ne 0)..           s(i,j,k) - s(i,j,k-1) =l= 13*h*(1-sum(l, p_c(l,k))) + 13*3*sum(l, p_c(l,k));
$ontext
The first constraint ensures vehicles moving forward.
xThe second constraint essentially penalizes significant changes in position when there are frequent or significant signal phase transitions.
$offtext

*A relaxed version of Equation (23b).
car_following(i,j,k)$(vehicle_indi(i,j) and veh_dynamics(i,j) ne 0)..            s(i,j,k) - s(i+1,j,k) =g= d_0;

$ontext
Equation (23c). Note that the zero point of s(i,j,k) is set to be the intersection point. Vehicles on incoming lanes have negative position values.
The following two equations can be derived if we divide Equation (23c) to two equations (left as one and right as the other one) and take negative on both sides.
$offtext
vehicle_position_1(i,j,k)$vehicle_indi(i,j)..                                    700*g(i,j,k) =g= s(i,j,k);
vehicle_position_2(i,j,k)$vehicle_indi(i,j)..                                    s(i,j,k) =g= 700*(g(i,j,k) - 1);

*Equation (23d).
traffic_rule(i,j,k)$(vehicle_indi(i,j) and k_dynamics(k) ne 0)..                 g(i,j,k) - g(i,j,k-1) - r(j,k) =l= 0;

$ontext
A more complex version of Equation (23f). Because we introduced p_c here. sum(l, p_c(l,k)) = 1 indicate there is a phase transition at time step k.
We need to add yellow time and all-red time during the transition.
First equ: if phase l has become activated in time step k (went from 0 to 1), the  transition in k-1 should be equal to 1
Second equ: Two steps in a row should not be tranition
Third equ: the phase should be yellow (sum(l,p(l,k))=0) when transition has happened
Fourth equ: the signal should not change for the next k steps because of pedestrian phase extension (not used for now so we set the threshold to 1)
$offtext
signal_rule_1(l,k)$(k_dynamics(k) ne 0)..                                        p(l,k) - p(l,k-1) - p_c(l,k-1) =l= 0;
signal_rule_2(k)$(k_dynamics(k) ne 0)..                                          sum(l, p_c(l,k)) + sum(l, p_c(l,k - 1)) =l= 1;
signal_rule_3(k)..                                                               sum(l, p(l,k)) + sum(l, p_c(l,k)) =e= 1;
signal_rule_4(l)..                                                               sum(k$(ord(k)<1), p_c(l,k)) =e= 0;




* Equation (23e)
phase_equal_1(k)..                                                               p('6',k) + p('8',k) =e= r('1',k);
phase_equal_2(k)..                                                               p('5',k) + p('6',k) =e= r('2',k);
phase_equal_3(k)..                                                               p('5',k) + p('6',k) =e= r('3',k);
phase_equal_4(k)..                                                               p('3',k) + p('4',k) =e= r('4',k);
phase_equal_5(k)..                                                               p('1',k) + p('3',k) =e= r('5',k);
phase_equal_6(k)..                                                               p('1',k) + p('3',k) =e= r('6',k);
phase_equal_7(k)..                                                               p('7',k) + p('8',k) =e= r('7',k);
phase_equal_8(k)..                                                               p('5',k) + p('7',k) =e= r('8',k);
phase_equal_9(k)..                                                               p('5',k) + p('7',k) =e= r('9',k);
phase_equal_10(k)..                                                              p('2',k) + p('4',k) =e= r('10',k);
phase_equal_11(k)..                                                              p('1',k) + p('2',k) =e= r('11',k);
phase_equal_12(k)..                                                              p('1',k) + p('2',k) =e= r('12',k);
ped_phase_equal_1(k)..                                                           p('1',k) + p('2',k) =e= q('1',k);
ped_phase_equal_2(k)..                                                           p('5',k) + p('6',k) =e= q('2',k);
ped_phase_equal_3(k)..                                                           p('1',k) + p('3',k) =e= q('3',k);
ped_phase_equal_4(k)..                                                           p('5',k) + p('7',k) =e= q('4',k);

*Equation (23h)
s.up(i,j,k)$vehicle_indi(i,j)= 700;
s.lo(i,j,k)$vehicle_indi(i,j) = -300;
s.fx(i,j,"1")$vehicle_indi(i,j) = s_init(i,j);
p.fx(l,'1') = p_init(l);

model mo /cost, cost_throughput, cost_distance, cost_transition, cost_delay, cost_ped_throughput, vehicle_dynamics_1, vehicle_dynamics_2, car_following, vehicle_position_1, vehicle_position_2, traffic_rule, signal_rule_1, signal_rule_2, signal_rule_3,signal_rule_4, phase_equal_1, phase_equal_2, phase_equal_3, phase_equal_4, phase_equal_5, phase_equal_6, phase_equal_7, phase_equal_8, phase_equal_9, phase_equal_10, phase_equal_11, phase_equal_12, ped_phase_equal_1, ped_phase_equal_2, ped_phase_equal_3, ped_phase_equal_4/;
*model mo /cost, cost_throughput, cost_distance, cost_transition, cost_delay, cost_ped_throughput, vehicle_dynamics_1, vehicle_dynamics_2, car_following, vehicle_position_1, vehicle_position_2, traffic_rule, signal_rule_1, signal_rule_2, signal_rule_3,signal_rule_4, phase_equal_1, phase_equal_2, phase_equal_3, phase_equal_4, phase_equal_5, phase_equal_6, phase_equal_7, phase_equal_8, phase_equal_9, phase_equal_10, phase_equal_11, phase_equal_12/;

*option optcr=0;
*option minlp=BONMIN;
mo.optfile = 1;
solve mo using minlp minimizing f;
model_status = mo.Modelstat;
solver_status = mo.solvestat;
