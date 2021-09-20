function v = getSpringVec(q, model, legs)
v = [];
    for i = 1:length(legs)
        if legs(i) == 1 % front leg
            foot = model.getFootPosition(q, 3);
            hip  = model.getPosition(q,1,model.hipLoc{1});
        elseif legs(i) == 2
            foot = model.getFootPosition(q, 1);
            hip  = model.getPosition(q,1,model.hipLoc{2});
        end      
        v = [v, foot - hip];
    end
end