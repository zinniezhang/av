sleep(0)
        
        @return_if_told(EMG)
       
       ego_meas = @fetch_or_continue(SENSE)
       fleet_meas = @fetch_or_continue(SENSE_FLEET)
       speed = ego_meas.speed


       # update the state of the road 
        for (id, f) ∈ fleet_meas
            x = f.position
            y = ego_meas.position
            dist = sqrt(sum((x - y) .^ 2))

            
            f_dist = x - center
            ego_dist = y - center
            θ_fleet = atan(f_dist[1], f_dist[2])
            θ_ego = atan(ego_dist[1], ego_dist[2])

            # if in front and close
            if angular_dist(θ_fleet, θ_ego) < 0 && dist < clear_dist[f.target_lane] 
                clear_dist[f.target_lane] = dist
                clear_speed[f.target_lane] = f.speed
            end

        end

        # new stuff
        # all clear
        if sum(clear_dist) == 60
            V = 15
            target_lane = ego_meas.target_lane

            # 1 clear
        elseif clear_dist[1] == 20
            V = 15
            target_lane = 1

            # 2 clear
        elseif clear_dist[2] == 20
            V = 15
            target_lane = 2

            # 3 clear
        elseif clear_dist[3] == 20
            V = 15
            target_lane = 3
        
            # none clear
        else
            maxVal, point = findmax(clear_dist)
            index = Int(point[2])
            target_lane = index
            V = clear_speed[target_lane]
        end

        #Newnew
         # all clear, stay in the middle lane
         if sum(clear_dist_front) == 3 * lane_margin
            #println("all clear, speed up")
            V = max_speed
            target_lane = 2

            # 2 clear
        elseif clear_dist_front[2] == lane_margin
            #println("2 clear, speed up")
            V = max_speed
            target_lane = 2
        
            # 1 clear
        elseif clear_dist_front[1] == lane_margin
            V = max_speed
            target_lane = 1

            # 3 clear
        elseif clear_dist_front[3] == lane_margin
            V = max_speed
            target_lane = 3
          
        else
            # front is blocked

            # lane 2 back is not blocked
            if clear_dist_back[2] == lane_margin
                target_lane = 2
                V = clear_speed_front[2] - 1
            
            elseif clear_dist_back[1] == lane_margin
                target_lane = 1
                V = clear_speed_front[1] - 1

            elseif clear_dist_back[3] == lane_margin
                target_lane = 3
                V = clear_speed_front[3] - 1

            end

            #100-105
            # maxVal, point = findmax(clear_dist)
            # index = Int(point[2])
            # target_lane = index
            # V = clear_speed[target_lane]
        end

        # check if changing 2 lanes
        if my_lane == 3 && target_lane == 1
            if clear_dist_front[2] == lane_margin && clear_dist_back[2] == lane_margin
                println("change 2")
                target_lane = 2
                V = clear_speed_front[2]
            else
                # stay in my_lane
                V = clear_speed_front[my_lane]
                target_lane = my_lane
            end

        elseif my_lane == 1 && target_lane == 3
            if clear_dist_front[2] == lane_margin && clear_dist_back[2] == lane_margin
                println("change 2")
                target_lane = 2
                V = clear_speed_front[2]
            else
                # stay in my_lane
                V = clear_speed_front[my_lane]
                target_lane = my_lane
            end
        end
