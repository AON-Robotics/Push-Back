% Calculates the analytical formulas for the Trapezoidal Time-scale

%% Initial Setup
clear      % Removes all variables, globals, and funtions from the workspace 
close all  % Closes remaining plots and graphs
clc        % Clears any text left in the command window.


%% Configuration
fprintf("1) Configuration\n");

evaluate_at_end = 1;
display_equations_at_end = 0;


%% Symbolic Constants & Variable Declarations
fprintf("2) Symbolic Constants & Variable Declarations\n");

syms t real             % Current time of motion
syms T real             % Total motion time

syms D real             % Desired distance to travel in motion.

syms V real             % Motion's maximum velocity (not always attainable)
syms v_0 real           % Motion's initial velocity
syms v_f real           % Motion's final velocity

syms a_0 real           % Motion's initial acceleration
syms a_f real           % Motion's final acceleration

% Time steps:
t_0 = (V - v_0) / a_0;  % Acceleration time step
t_f = (V - v_f) / a_f;  % Decceleration time step
t_v = T - t_0 - t_f;    % Constant velocity time step

%% Helper methods

enhanced_simplify = @(f) collect(simplify(f, 'Steps', 100, 'Seconds', 10, 'IgnoreAnalyticConstraints', true, 'Criterion', 'preferReal'), t);

enhanced_solve = @(eq, var) solve(eq, var, 'ReturnConditions', true, 'IgnoreAnalyticConstraints', true, 'IgnoreProperties', false, 'Real', true);

enhanced_expand = @(f) expand(f, 'IgnoreAnalyticConstraints', true);

%% Assumptions for Symbolic Constants & Variables
fprintf("3) Setting Assumptions for Symbolic Constants & Variables\n");

assume(0 < T);                  % `T` is positive
assumeAlso(0 <= t & t <= T);    % Time is positive and less than `T`

assumeAlso(0 < D);              % `D` is positive
assumeAlso(0 < V);              % `V` is positive

assumeAlso(0 < a_0 & 0 < a_f);  % Accelerations are positive.
assumeAlso(v_0 < V & v_f < V);  % Initial and final speeds are smaller than V.
assumeAlso(0 <= t_0 & 0 <= t_f & 0 <= t_v); % All time steps are positive.


%% Calculations
fprintf("4) Calculations\n");

fprintf("  a) Acceleration\n");
a_1(t) = a_0;
a_2(t) = 0*t;
a_3(t) = -a_f;
a(t) = piecewise(                      ...
    (0 <= t & t < t_0), a_1(t),        ...
    (t_0 <= t & t < T - t_f), a_2(t),  ...
    (T - t_f <= t & t <= T), a_3(t),   ...
    0);

fprintf("  b) Speed\n");
% Speed is the integral of accelration.
v_1(t) = enhanced_simplify(int(a_1, 0, t) + v_0);
v_2(t) = enhanced_simplify(0 + v_1(t_0));
v_3(t) = enhanced_simplify(int(a_3, T - t_f, t) + v_2(T - t_f));

v(t) = piecewise(                       ...
    (0 <= t & t < t_0), v_1(t),         ...
    (t_0 <= t & t < T - t_f), v_2(t),   ...
    (T - t_f <= t & t <= T), v_3(t),    ...
    0);

fprintf("  c) Distance\n");
% Distance is the integral of accelration.
d_1(t) = enhanced_simplify(int(v_1, t, 0, t));
d_2(t) = enhanced_simplify(int(v_2, t, t_0, t) + d_1(t_0));
d_3(t) = enhanced_simplify(int(v_3, t, t_0 + t_v, t) + d_2(T - t_f));

d(t) = piecewise((0 <= t & t < t_0), d_1(t),   ...
    (t_0 <= t & t < T - t_f), d_2(t),          ...
    (T - t_f <= t & t <= T), d_3(t),           ...
    0);

% Total distance travelled by the end of the motion
D_calc = enhanced_simplify(d_3(T));

% Calculate T and other constraints on variables
fprintf("  d) Time Constraints\n");
soln = enhanced_solve(D == D_calc, T);

T_calc = enhanced_simplify(enhanced_expand(soln.T));

%% Find constraints for `V`
fprintf("5) Finding constraints for `V`\n");

% Convert soln.conditions into an array of symbolic expressions.
T_conds = str2sym(split(string(simplify(soln.conditions)), " & "));

% If nothing changes, T_conds(2) and T_conds(3) will be true for all real
% numbers.
soln = enhanced_solve(T_conds(1), V);

assumeAlso(subs(soln.conditions, soln.parameters(1), V));


%% Display equations
if display_equations_at_end
    fprintf("6) Displaying equations\n\n");
    
    fprintf("a) Speed\n");
    pretty(v);
    
    fprintf("\nb) Distance\n");
    pretty(d);
    
    fprintf("\nc) T\n");
    pretty(T_calc);
    
    fprintf("\nd) V_max\n");
    pretty( V <= ((a_f*v_0^2 + a_0*v_f^2 + 2*D*a_0*a_f)/(a_0 + a_f))^(1/2));
    
else
    fprintf("6) NOT Displaying equations\n")
    
end 

%% Analysis after previous calculations

if evaluate_at_end ~= 0
    % Constrain V based on `T_conds`
    assumeAlso(V <= sqrt((2*D*a_0*a_f + v_0^2*a_f + v_f^2*a_0) / (a_0 + a_f)));
    % * Fortunately agrees with the constraint that t_v > 0
    % This makes sure that there is always continuity in the function
    % The second and third `T_conds` seemed unnecessary (for now)

    % Evaluating
    D_v = 2;
    V_v = 1;
    v_0_v = 0;
    v_f_v = 0;
    a_0_v = 1;
    a_f_v = 1;

    T_v = double(subs(D/V + (V - v_0)^2/(2*V*a_0) + (V - v_f)^2/(2*V*a_f), [D, V, v_0, v_f, a_0, a_f], [D_v, V_v, v_0_v, v_f_v, a_0_v, a_f_v]));
    velocity(t) = subs(v, [D, V, v_0, v_f, a_0, a_f, T], [D_v, V_v, v_0_v, v_f_v, a_0_v, a_f_v, T_v]);
    distance(t) = subs(d, [D, V, v_0, v_f, a_0, a_f, T], [D_v, V_v, v_0_v, v_f_v, a_0_v, a_f_v, T_v]);

    hold on
    fplot(velocity, [0, T_v]);
    fplot(distance, [0, T_v]);
    hold off
end

fprintf("Finished!!\n");