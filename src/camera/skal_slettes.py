            if robot_head and robot_tail:
                robot_center = (
                    (robot_head["pos"][0] + robot_tail["pos"][0]) // 2,
                    (robot_head["pos"][1] + robot_tail["pos"][1]) // 2
                )
                print("[DEBUG] robot_center:", robot_center)  # <--- DEBUG

            # Ball counting logic
            current_visible_balls = len(balls)
            if 'previous_ball_count' not in locals():
                previous_ball_count = current_visible_balls
            if current_visible_balls < previous_ball_count:
                balls_difference = previous_ball_count - current_visible_balls
                current_run_balls += balls_difference
                total_balls_collected += balls_difference
                print("*** BALL(S) COLLECTED! Run: {}/{}, Total: {}/{} ***".format(
                    current_run_balls, STORAGE_CAPACITY, total_balls_collected, TOTAL_BALLS_ON_COURT))
            previous_ball_count = current_visible_balls

            navigation_info = None

            if current_state == COLLECTING:
                print("[DEBUG] State: COLLECTING")
                print("[DEBUG] current_run_balls:", current_run_balls, "balls:", balls)
                if current_run_balls >= STORAGE_CAPACITY:
                    current_state = DELIVERING
                    print("*** STORAGE FULL - SWITCHING TO GOAL DELIVERY ***")
                    route_manager.reset_route()
                elif not balls and current_run_balls > 0:
                    current_state = DELIVERING
                    print("*** NO MORE BALLS ON FIELD - DELIVERING WHAT'S COLLECTED ***")
                    route_manager.reset_route()
                elif not balls and current_run_balls == 0 and total_balls_collected >= TOTAL_BALLS_ON_COURT:
                    current_state = COMPLETE
                    print("*** ALL BALLS COLLECTED - MISSION COMPLETE ***")
                    route_manager.reset_route()
                elif balls and current_run_balls < STORAGE_CAPACITY:
                    # Route-based navigation for balls
                    remaining_balls = [b for b in balls if b not in collected_balls_positions]
                    print("[DEBUG] remaining_balls:", remaining_balls)
                    route_manager.create_route_from_balls(remaining_balls, robot_center, walls, cross_pos)
                    target_ball = route_manager.get_current_target()
                    print("[DEBUG] target_ball:", target_ball)
                    if target_ball:
                        target_ball = route_manager.get_wall_approach_point(target_ball, walls, scale_factor)
                        navigation_info = calculate_navigation_command(robot_head, robot_tail, target_ball, scale_factor)
                        handle_robot_navigation(navigation_info, commander, route_manager)
                elif not balls and current_run_balls > 0:
                    current_state = DELIVERING
                    print("*** NO MORE BALLS ON FIELD - DELIVERING WHAT'S COLLECTED ***")
                    route_manager.reset_route()

        
            elif current_state == DELIVERING:
                print("[DEBUG] State: DELIVERING") # <--- DEBUG
                goal_position = goal_utils.get_goal_position()
                print("[DEBUG] goal_position:", goal_position)  # <--- DEBUG
                if goal_position:
                    navigation_info = calculate_navigation_command(robot_head, robot_tail, goal_position, scale_factor)
                    print("Navigation info to goal:", navigation_info) # <--- DEBUG
                    if navigation_info:
                        print("[DEBUG] Calling handle_robot_navigation for goal")  # <--- DEBUG
                        handle_robot_navigation(navigation_info, commander, route_manager)
                    
                    # Check if close enough to goal to finish delivery
                    if navigation_info and navigation_info.get("distance_cm", 999) < 5:
                        print("*** REACHED GOAL - READY TO RELEASE ***")
                        commander.send_release_balls_command(duration=4)
                        current_run_balls = 0
                        if total_balls_collected >= TOTAL_BALLS_ON_COURT:
                            current_state = COMPLETE
                            print("*** ALL BALLS DELIVERED - MISSION COMPLETE ***")
                        else:
                            current_state = COLLECTING
                            print("*** BALLS RELEASED - STARTING NEW COLLECTION RUN ***")
                        route_manager.reset_route()
                else:
                    print("ERROR: No goal position set - cannot navigate to goal!")

            elif current_state == COMPLETE:
                print("[DEBUG] State: COMPLETE")  # <--- DEBUG
                if commander.can_send_command():
                    print("*** MISSION COMPLETE - STOPPING ROBOT ***")
                    commander.send_stop_command()