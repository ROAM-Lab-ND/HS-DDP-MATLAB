function [J,Jd] = Link5Jacobian(in1,in2)
%LINK5JACOBIAN
%    [J,JD] = LINK5JACOBIAN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    29-Nov-2020 17:42:53

p1_1 = in2(1);
p2_1 = in2(2);
q3 = in1(3,:);
q6 = in1(6,:);
q7 = in1(7,:);
qd3 = in1(10,:);
qd6 = in1(13,:);
qd7 = in1(14,:);
t2 = sin(q3);
t3 = cos(q3);
t4 = sin(q6);
t5 = cos(q6);
t6 = cos(q7);
t7 = t2.*t4;
t15 = t3.*t5;
t8 = t7-t15;
t9 = sin(q7);
t10 = t3.*t4;
t11 = t2.*t5;
t12 = t10+t11;
t13 = t2.*t4.*(2.09e2./1.0e3);
t14 = conj(p1_1);
t16 = t8.*t9;
t22 = t6.*t12;
t27 = t16-t22;
t17 = t14.*t27;
t18 = conj(p2_1);
t19 = t6.*t8;
t20 = t9.*t12;
t21 = t19+t20;
t23 = t3.*t4.*(2.09e2./1.0e3);
t24 = t2.*t5.*(2.09e2./1.0e3);
t25 = t14.*(t19+t20);
t26 = t18.*(t16-t22);
t28 = t23+t24+t25+t26;
t29 = t3.*(1.9e1./1.0e2);
t30 = t23+t24+t25+t26+t29;
t31 = t14.*t21;
t32 = t18.*t27;
t33 = qd7.*(t31+t32);
t34 = qd6.*t28;
t36 = t18.*t21;
t35 = t17-t36;
t37 = t2.*(1.9e1./1.0e2);
J = reshape([1.0,0.0,0.0,1.0,t13+t17+t37-t3.*t5.*(2.09e2./1.0e3)-t18.*t21,t30,0.0,0.0,0.0,0.0,t13+t17-t3.*t5.*(2.09e2./1.0e3)-t18.*t21,t28,t35,t25+t26],[2,7]);
if nargout > 1
    t39 = t3.*t5.*(2.09e2./1.0e3);
    t38 = t13+t17-t36-t39;
    Jd = reshape([0.0,0.0,0.0,0.0,t33+t34+qd3.*t30,-qd7.*t35-qd6.*t38-qd3.*(t13+t17-t36+t37-t3.*t5.*(2.09e2./1.0e3)),0.0,0.0,0.0,0.0,t33+t34+qd3.*t28,-qd3.*t38-qd7.*t35-qd6.*t38,t33+qd3.*(t31+t32)+qd6.*(t31+t32),-qd3.*t35-qd6.*t35-qd7.*t35],[2,7]);
end