function v = TargetCAVF(Xf, P)
    %v = (Xf - P) * norm(Xf - P)^(-0.5);
    v = Xf - P;
end