function [zeroCrossing,isterminal,direction] = event_touchDown(t,X,p)

params = p.params;

q = X(1:4);

p_toe = fcn_p_toe(q,params);


zeroCrossing =  p_toe(3);
isterminal   =  1;
direction    =  -1;




