clear; clc;

% Numero de joints del robot
num = 2; %cambiar este numero segun joints del robot

syms q1 q2
syms l1 l2
syms g

pq = [q1 q2];

q = [q1 q2];
alpha = [0 0];
d = [0 0];
a = [l1 l2];

Rk0 = 1;
z = sym('z', [3 num]);
sigma = sym('sigma', [3 num+1]);

J = sym('j', [6 num]);
J_v = sym("x", [3 num]);
J_w = sym("x", [3 num]);
I = sym("I", [1 num]);
m = sym("m", [1 num]);
qd = sym("qd", [1 num]);

J_vsigma= sym('x', [3 num]);

%% Rotaciones

for n = 1:num
    Rn = [cos(q(n)) -sin(q(n))*cos(alpha(n)) sin(q(n))*sin(alpha(n)) a(n)*cos(q(n));
         sin(q(n)) cos(q(n))*cos(alpha(n)) -cos(q(n))*sin(alpha(n)) a(n)*sin(q(n));
         0 sin(alpha(n)) cos(alpha(n)) d(n);
         0 0 0 1];
    
    for i = 1:3
        for j = 1:3
            coef =  coeffs(Rn(i, j));
            if ~isempty(coef)
                if coef > -0.5 && coef < 0.5
                    Rn(i, j) = 0;
                else
                    Rn(i,j) = Rn(i,j)/abs(coef);
                end
            end
        end
    end
    
    Rk0 = Rk0*simplify(Rn);
    Rk0 = simplify(Rk0);
    
    for i = 1:3
        sigma(i, n+1) = Rk0(i, 4);
        z(i, n) = Rk0(i, 3);
    end
end

sigma(1:3, 1) = [0 0 1]';
% disp(Rk0)

%% Jacobianos
inputArray = 'a'*num;

for i = 1:num
    inp = input('Joint revoluta o prismatica: R/P', 's');
    inputArray(i) = inp;
    switch inp
         case 'P'
            for j = 1:3
                J_v(j, i) = z(j, i);
                J_vsigma(j, i) = z(j, i);
                J_w(j, i) = 0;
            end
        case 'R'
            aux = cross(z(:, i), (sigma(:, num + 1) - sigma(:, i)));
            if i == 1
                aux2 = cross(z(:, i),sigma(:, i+1));
            else
                aux2 = aux;
            end
            
            for j = 1:3
                J_v(j, i) = aux(j);
                J_vsigma(j, i) = aux2(j);
                J_w(j, i) = z(j, i);
            end
            
        otherwise
            fprintf('Please input something correct')
    end
end

J_vsigma = [[0; 0; 0;] J_vsigma];
J_vc_aux = J_vsigma(:, 2)/num;

for i = 3:num+1
    for j = i-1:-1:1
        J_vc_aux = [J_vc_aux (J_vsigma(:, j) + J_vsigma(:, i)/num)];
    end
end


J_vc = J_vc_aux(:, 1);
for i = 1:num
    for j = i:i+i-1
        J_vc = [J_vc J_vc_aux(:, j)];
    end

    for k = 1:num-i
        J_vc = [J_vc [0; 0; 0]];
    end
end

J_vc = J_vc(:, 2:num^2+1);

for i = 1:num
    switch inputArray(i)
        case 'P'
            for j = 1:3
                J(j, i) = J_v(j, i);
                J(j+3, i) = 0;
            end
        case 'R'
            for j = 1:3
                J(j, i) = J_v(j, i);
                J(j+3, i) = J_w(j, i);
            end
        otherwise
            fprintf('Please input something correct')
    end
end

%% Matriz de inercias y Coriolis

D = m(1).*(J_vc(:, 1:num)).'*J_vc(:, 1:num);
for i = 1:(num-1)
    D = D + m(i+1).*(J_vc(:, (i*num)+1:(i+1)*num)).'*J_vc(:, (i*num)+1:(i+1)*num); %% Falta sumar inercias
end

D = simplify(D);

coriolis = sym('x', [num num]);

for k = 1:num
    for j = 1:num
        for i = 1:num
            if i == 1
                coriolis(k, j) = (1/2)*(diff(D(k, j),pq(i)) + diff(D(k, i), pq(j)) - diff(D(i,j), pq(k))) * qd(i);
            else
                coriolis(k, j) = coriolis(k, j) + (1/2)*(diff(D(k, j),pq(i)) + diff(D(k, i), pq(j)) - diff(D(i,j), pq(k))) * qd(i);
            end
        end
        coriolis(k, j) = simplify(coriolis(k, j));
    end
end

coriolis = simplify(coriolis);

%% Energia potencial
p = m(1)*g*-(J_vc_aux(1, 1));
for i = 1:num-1
    p = p + m(i+1)*g*-(J_vc_aux(1, i*num));
end

gP = sym('x', [1 num]);
for i = 1:num
    gP(i) = diff(p, pq(i));
end
disp(p)
disp(gP)

%% Tau
Kp = sym('Kp', [num num]);
Kd = sym('Kd', [num num]);
dq = sym('qd', [num 1]);

tildeq = pq' - dq;

tau = -Kp*tildeq - Kd*(qd).' + gP;

ddq = D\(tau - coriolis*(qd).' -gP);

y = [ddq; tau; gP];