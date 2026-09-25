from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=False)

    sl.node('racer_cpp', 'racer_exec', prefix='xterm -hold -e')

    sl.node('racer_cpp', 'obstacle_clustering')#, prefix='xterm -hold -e')

    sl.node('racer_cpp', 'camera_rgb_wall_filter')#, prefix='xterm -hold -e')

   # sl.node('racer_cpp', 'smppi', prefix='xterm -hold -e')

   # sl.node('racer_cpp', 'occupancy_grid_list')

    #sl.node('racer_cpp', 'odometry')


    return sl.launch_description()
