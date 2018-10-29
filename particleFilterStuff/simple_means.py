def estimate_pose(pose_array):
    # takes input in the form of a ROS PoseArray
    # currently averaging values
    # averaging quanternions only makes sense if their orientations are similar. If they're not, averages are
    # meaningless and require multiple representations (from paper below).
    # http://www.cs.unc.edu/techreports/01-029.pdf

    x,y,qy,qw = 0,0,0,0

    for item in pose_array.poses:
        # z should always be 0
        x += item.position.x
        y += item.position.y

        # this should need yaw only
        qy += item.orientation.y
        qw += item.orientation.w


    n = len(pose_array.poses)

    # calculate mean values
    mean_x = x/n
    mean_y = y/n

    mean_qy = qy/n
    mean_qw = qw/n

    # the other vars should be initialised to 0.0 so don't need to be defined here
    estimated_pose = Pose()
    estimated_pose.position.x = mean_x
    estimated_pose.position.y = mean_y

    estimated_pose.orientation.y = mean_qy
    estimated_pose.orientation.w = mean_qw

    return estimated_pose