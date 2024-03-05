#f/usr/bin/env python3
import rospy
from std_msgs.msg import String

class PlayerA:
    def __init__(self):
        rospy.init_node('player_a')
        self.server_topic = '/server'
        self.player_topic = '/player_a'
        self.monster_names = ['Fire', 'Water', 'Earth']
        self.attack_moves = []
        self.current_state = 'WAITING'
        self.next_state = 'WAITING'

    def send_attack_moves(self):
        moves = []
        for monster in self.monster_names:
            move = input(f"Enter attack move for {monster}: ")
            moves.append(move)
        self.attack_moves = moves
        rospy.loginfo(f"Sending attack moves: {self.attack_moves}")
        rospy.Publisher(self.player_topic, String, queue_size=1).publish(','.join(self.attack_moves))

    def server_callback(self, msg):
        hitpoints = msg.data.split(',')
        rospy.loginfo(f"Received hitpoints from the server: {hitpoints}")
        if self.current_state == 'WAITING':
            self.current_state = 'RECEIVING_MOVES'

    def run(self):
        rospy.Subscriber(self.server_topic, String, self.server_callback)
        while not rospy.is_shutdown():
            if self.current_state == 'WAITING':
                rospy.sleep(1)
            elif self.current_state == 'RECEIVING_MOVES':
                self.send_attack_moves()
                self.next_state = 'WAITING'
                rospy.sleep(1)
                self.current_state = self.next_state

if __name__ == '__main__':
    player_a = PlayerA()
    player_a.run()
