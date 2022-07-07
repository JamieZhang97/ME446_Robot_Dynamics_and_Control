function [zeroCrossing,isterminal,direction] = event_liftOff(t,X,p)

Tst = p.Tst;
tTD = p.tTD;

zeroCrossing =  t - tTD - Tst;
isterminal   =  1;
direction    =  1;




