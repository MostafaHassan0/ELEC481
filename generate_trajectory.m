function r = generate_trajectory(t, type, f, Amplitude)

if (type == 1)
    %step 
    r = Amplitude *ones(1, numel(t)); 
elseif (type == 2)
    %square wave
    r = Amplitude * sign(sin(2*pi*f*t)); 
elseif (type == 3)
    r = Amplitude .* sin(2*pi*f*t); 
else
      error('Unknown trajectory type.'); 
end
r = [r; r]; 

end