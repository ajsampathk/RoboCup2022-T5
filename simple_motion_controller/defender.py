import rospy
import move_bot3


if __name__ == '__main__':

    defender1 = move_bot3.robot("nao13")

    ball_x = move_bot3.ball_current_state.pose.position.x
    ball_y = move_bot3.ball_current_state.pose.position.y

    if ball_x < 0 and ball_y < 0 :
