

selectedState = states(:,4);

feet1 = tform2trvec(getTransform(robot,selectedState,"feet1"));
feet2 = tform2trvec(getTransform(robot,selectedState,"feet2"));
feet3 = tform2trvec(getTransform(robot,selectedState,"feet3"));
feet4 = tform2trvec(getTransform(robot,selectedState,"feet4"));

drawCenterMass(robot,selectedState,feet4,feet2,feet3);

function drawCenterMass(robot,hConfig,feet1, feet2, feet3)
    show(robot,hConfig,'Frames','off');
    [comLocation,comJac] = centerOfMass(robot,hConfig);
    hold on;
    scatter3(comLocation(1),comLocation(2),comLocation(3),'filled');
    line([feet1(1),feet2(1),feet3(1),feet1(1)], ...
    [feet1(2),feet2(2),feet3(2),feet1(2)], ...
    [feet1(3),feet2(3),feet3(3),feet1(3)]);
    hold off;
end