function [ vel, delta ] = sense_model( obj )
    
    vel     = normrnd(obj.vel,  obj.enc_err); 
    delta   = normrnd(obj.delta,obj.str_err); 
    
end