function y = simndof(input)
    q1=input(1);
    q2=input(2);
    dq1=input(3);
    dq2=input(4);
    t=input(5);
    q=[q1;q2];
    dq=[dq1;dq2];
    
    %constantes 
    l1=1; lc1=l1/2; l2=2; lc2=l2/2; m1=2; m2=4; I1=0.001; I2=0.002; g=9.81;
    
    
    D =[ m2*l1^2 + 2*m2*cos(q2)*l1*lc2 + m1*lc1^2 + m2*lc2^2 + I1 + I2, m2*lc2^2 + l1*m2*cos(q2)*lc2 + I2;
                                 m2*lc2^2 + l1*m2*cos(q2)*lc2 + I2,                     m2*lc2^2 + I2];
    
                                       
    h=-m2*l1*lc2*sin(q2);
    
    C=[h*dq2 h*(dq2+dq1);
       -h*dq1 0];
    
    nablaP = [g*m2*(lc2*cos(q1 + q2) + l1*cos(q1)) + g*lc1*m1*cos(q1);
                                       g*lc2*m2*cos(q1 + q2)];
     
     Kp=[7 0; 0 3];
     Kd=[3 0; 0 6];
     qd=[pi/2; -pi/4];
     tildeq=q-qd;
     
     tau=-Kp*tildeq-Kd*dq+nablaP;  %%PD+comp. graved
                                   
                                   
    ddq=D\(tau-C*dq-nablaP);
    
    y=[ddq;tau;tildeq];
end