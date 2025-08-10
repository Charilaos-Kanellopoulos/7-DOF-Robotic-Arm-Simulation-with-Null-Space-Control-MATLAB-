function Li = L(theta,dz)
    Li = zeros(length(theta),1);
    for i=1:length(theta)
        zi = (dz(i,2)+dz(i,1))/2; 
        Li(i) = ((theta(i)-zi)/(zi-dz(i,2)))^2;
    end
end