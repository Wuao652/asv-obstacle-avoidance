import numpy as np
import matplotlib.pyplot as plt
import pickle
ROBOT_H = 6.0
ROBOT_W = 2.8

with open('sac_test.pkl', 'rb') as f:
    robot_dict = pickle.load(f)

# target_goal = np.array([[13.2, -42.0, 0.0],
#                            [19.6, -33.0, 0.0],
#                            [10.9, -24.3, 0.0],
#                            [23.0, -58.0, 0.0]])

target_goal = np.array([[13.2, -42.0, 0.0],
                           [22.6, -33.0, 0.0],
                           [10.9, -24.3, 0.0],
                           [23.0, -58.0, 0.0],
                           [0.0, 0.0, 0.0]])

obstacle_position = np.array([[-2.0, -16.9, 0.0],
                              [1.0,  -15.3, 0.0],
                              [22.0, -16.8, 0.0],
                              [19.0, -16.2, 0.0],
                              [16.0, -15.1, 0.0],
                              [-2.0, -33.5, 0.0],
                              [1.0,  -32.7, 0.0],
                              [4.0,  -32.3, 0.0],
                              [7.0,  -32.4, 0.0],
                              [10.0, -33.8, 0.0],
                              [13.0, -49.5, 0.0],
                              [16.0, -49.6, 0.0],
                              [19.0, -48.8, 0.0],
                              [22.0, -49.3, 0.0],
                              [-2.0, -48.7, 0.0]])

if __name__ == '__main__':
    print("hello world from asv_vis!")

    fig = plt.figure()
    ax = fig.gca()

    # draw obstacle
    for i in range(obstacle_position.shape[0]):

        safe_zone = plt.Circle((obstacle_position[i][0], obstacle_position[i][1]),
                            radius=1.5,
                            facecolor='none',
                            edgecolor='r',
                            ls='--')
        ob = plt.Circle((obstacle_position[i][0], obstacle_position[i][1]),
                        radius=0.5,
                        color='r')

        ax.add_patch(safe_zone)
        ax.add_patch(ob)

    robot_position = robot_dict[3]
    #draw robots
    for i in range(robot_position.shape[0]):
        x = robot_position[i][0]
        y = robot_position[i][1]
        theta = robot_position[i][2]
        T = np.array([[np.cos(theta), -np.sin(theta), x],
                      [np.sin(theta), np.cos(theta), y],
                      [0.0, 0.0, 1.0]])
        p_r = np.array([-3, -1.4, 1])
        p_w = T @ p_r

        x1, y1 = p_w[0], p_w[1]

        robot = plt.Rectangle((x1, y1),
                              ROBOT_H,
                              ROBOT_W,
                              theta / np.pi * 180,
                              facecolor='blue',
                              edgecolor='black')
        ax.add_patch(robot)

    # draw start point
    start_width = 2
    start = plt.Rectangle((0 - start_width / 2, -5 - start_width / 2),
                           start_width,
                           start_width)
    ax.add_patch(start)
    # draw target point
    target_x, target_y = target_goal[3][0], target_goal[3][1]
    target_zone = plt.Circle((target_x, target_y),
                           radius=3,
                           facecolor='none',
                           edgecolor='g',
                           ls='--')
    ax.add_patch(target_zone)
    target_width = 2
    target = plt.Rectangle((target_x - target_width / 2, target_y - target_width / 2),
                           target_width,
                           target_width,
                           color='g')
    ax.add_patch(target)

    # draw boundary
    plt.axline((-5, -5), (-5, -65), color='black', lw='4')
    plt.axline((25, -5), (-5, -5), color='black', lw='4')
    plt.axline((25, -5), (25, -65), color='black', lw='4')
    plt.axline((25, -65), (-5, -65), color='black', lw='4')
    plt.xlim(-5, 25)
    plt.ylim(-5, -65)

    plt.gca().set_aspect('equal', adjustable='box')
    plt.axis('off')
    plt.show()







