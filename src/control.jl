function wrap(θ)
    mod(θ += pi, 2*pi) - pi
end

function clip(x, l)
    max(-l, min(l, x))
end

function keyboard_controller(KEY::Channel, 
                             CMD::Channel, 
                             SENSE::Channel, 
                             EMG::Channel;
                             K1=5, 
                             K2=.5, 
                             disp=false, 
                             V=0.0, 
                             θ = 0.0, 
                             V_max = 100.0, 
                             θ_step=0.1, 
                             V_step = 1.5)
    println("Keyboard controller in use.")
    println("Press 'i' to speed up,")
    println("Press 'k' to slow down,")
    println("Press 'j' to turn left,")
    println("Press 'l' to turn right.")

    while true
        sleep(0)
        @return_if_told(EMG)
        
        key = @take_or_default(KEY, ' ')
        meas = @fetch_or_continue(SENSE)

        speed = meas.speed
        heading = meas.heading
        segment = meas.road_segment_id
        if key == 'i'
            V = min(V_max, V+V_step)
        elseif key == 'j' 
            θ += θ_step
        elseif key == 'k'
            V = max(0, V-V_step)
        elseif key == 'l'
            θ -= θ_step
        end
        err_1 = V-speed
        err_2 = clip(θ-heading, π/2)
        cmd = [K1*err_1, K2*err_2]
        @replace(CMD, cmd)
        if disp
            print("\e[2K")
            print("\e[1G")
            @printf("Command: %f %f, speed: %f, segment: %d", cmd..., speed, segment)
        end
    end
end

function angular_dist(θ_fleet, θ_ego)
    return mod(((θ_fleet - θ_ego) + π), 2π) - π

end

function controller(CMD::Channel, 
                    SENSE::Channel, 
                    SENSE_FLEET::Channel, 
                    EMG::Channel,
                    road)
    ego_meas = fetch(SENSE)
    fleet_meas = fetch(SENSE_FLEET)

    V=30
    θ=0.0
    K1=5
    K2=0.5
    K₁=0.5
    K₂=.5

    # False = 0 and True = 1 
    ego_meas = fetch(SENSE)
    fleet_meas = fetch(SENSE_FLEET)
    my_lane = ego_meas.target_lane
    target_lane = 2
    seg = road.segments[1]
    err_1 = 0
    center = seg.center
    lane_margin = 15.0

    max_speed = 30.0
    clear_dist_front = [lane_margin lane_margin lane_margin]
    clear_dist_back = [lane_margin lane_margin lane_margin]
    clear_speed_front = [max_speed max_speed max_speed]
    clear_speed_back = [max_speed max_speed max_speed]
    slow_down = 35.0

    println("")
    println("edit4")
    println("")

    # 1 is true 0 is false 
    while true
        sleep(0)
        
        @return_if_told(EMG)
       
       ego_meas = @fetch_or_continue(SENSE)
       fleet_meas = @fetch_or_continue(SENSE_FLEET)
       speed = ego_meas.speed

        # update road state
        for (id, f) ∈ fleet_meas
            x = f.position
            y = ego_meas.position
            dist = sqrt(sum((x - y) .^ 2))

            
            f_dist = x - center
            ego_dist = y - center
            θ_fleet = atan(f_dist[1], f_dist[2])
            θ_ego = atan(ego_dist[1], ego_dist[2])

            
            # Find car is close
            if dist < lane_margin 
                    # in front
                if angular_dist(θ_fleet, θ_ego) < 0 && dist < clear_dist_front[f.target_lane] 
                    clear_dist_front[f.target_lane] = dist
                    clear_speed_front[f.target_lane] = f.speed
                end
                
            end

           #=  if dist < lane_margin *1.5
                # behind
                if angular_dist(θ_fleet, θ_ego) > 0 && dist < clear_dist_back[f.target_lane] 
                    clear_dist_back[f.target_lane] = dist
                    clear_speed_back[f.target_lane] = f.speed
                end
            end =#
        
        end 

        if my_lane == 1
            
            # front of lane 1 clear
            if (clear_dist_front[1] == lane_margin)
                V = max_speed
                target_lane = 1
            
                # lane 2 completely clear, switch
            elseif (clear_dist_front[2] == lane_margin) && (clear_dist_back[2] == lane_margin)
                
                target_lane = 2
                V = minimum(clear_speed_front) - slow_down

            # stay in 1, follow car infront
            else
                V = clear_speed_front[1]
                target_lane = 1
       
            end

        elseif my_lane == 2

            # lane 2 completely clear, speed up
            if (clear_dist_front[2] == lane_margin)
                V = max_speed
                target_lane = 2
            
            # lane 1 clear, switch
            elseif (clear_dist_front[1] == lane_margin) && (clear_dist_back[1] == lane_margin)
                target_lane = 1
                V = minimum(clear_speed_front) - slow_down
                println(V)
            
            # lane 3 clear, switch
            elseif  (clear_dist_front[3] == lane_margin) && (clear_dist_back[3] == lane_margin)
                target_lane = 3
                V = minimum(clear_speed_front) - slow_down
                println(V)

            # stay in 2, follow car infront
            else 
                V = clear_speed_front[2]
                target_lane = 2

            end

        elseif my_lane == 3
            
            # front of lane 3 clear
            if (clear_dist_front[1] == lane_margin)
                V = max_speed
                target_lane = 3
            
            # lane 2 completely clear, switch
            elseif (clear_dist_front[2] == lane_margin) && (clear_dist_back[2] == lane_margin)
                target_lane = 2
                V = minimum(clear_speed_front) - slow_down
                print("switch to 2", V)

            # stay in 3, follow car infront
            else
                V = clear_speed_front[3]
                target_lane = 3
                println("stay in 3", V)
                println("dist front", clear_dist_front[3])
                println("speed front", clear_speed_front[3])
      
            end

        end

        err_1 = V - speed
    
        cte, ctv = get_crosstrack_error(ego_meas.position, ego_meas.heading, speed, target_lane, seg, road.lanes, road.lanewidth)
        δ = -K₁*cte-K₂*ctv
        cmd = [K1 * err_1 max(min(δ, π/4.0), -π/4.0)]
        @replace(CMD, cmd)

        # reset state
        clear_dist_front = [lane_margin lane_margin lane_margin]
        clear_dist_back = [lane_margin lane_margin lane_margin]
        clear_speed_front = [max_speed max_speed max_speed]
        clear_speed_back = [max_speed max_speed max_speed]
        my_lane = target_lane
    end
end
