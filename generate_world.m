function world = generate_world(obstical_count,spacing,width,max_height,settings)
    template.height = 0;
    template.position = 0;
    template.corners = 0;

    ooeoc = floor(sqrt(obstical_count));

    obsticals(1,obstical_count) = template;
    count = 1;
    for i = 1:ooeoc
        for j = 1:ooeoc
            
            height = rand*(-max_height);
            position = [spacing+width;spacing+width].*[i;j]-[spacing+width;spacing+width]*1.5;
            obsticals(count).corners = [position(1) + width/2,   position(1) - width/2,   position(1) - width/2,   position(1) + width/2,   position(1) + width/2,   position(1) - width/2,   position(1) - width/2,   position(1) + width/2;
                                    position(2) + width/2,   position(2) + width/2,   position(2) - width/2,   position(2) - width/2,   position(2) + width/2,   position(2) + width/2,   position(2) - width/2,   position(2) - width/2;
                                    0,                       0,                       0,                       0,                       height,                  height,                  height,                  height];                        
            count = count + 1;
        end
    end
    world.obsticals = obsticals;
    world.edges = settings.window;
end