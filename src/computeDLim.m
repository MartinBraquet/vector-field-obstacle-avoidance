function dLim = computeDLim(obj, P, V, uMax)
        % Inflate the obstacle boundary depending on the max input
        % control u
        VArel = V - obj.Vo; % relative agent velocity towards the obs (wrt the obstacle)
        nObsAgent = (P - obj.Po) / norm(P - obj.Po);
        VArel = VArel - dot(nObsAgent,VArel) * nObsAgent; % projection along line of sight between agent and obstacle
        dLim = norm(VArel)^2 / (2*uMax);
end