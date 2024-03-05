#f/usr/bin/env python3
import rospy
from std_msgs.msg import String

class GameModerator:
    def __init__(self):
        rospy.init_node('game_moderator')
        self.player_a_topic = '/player_a'
        self.player_b_topic = '/player_b'
        self.server_topic = '/server'
        self.player_a_hitpoints = [300, 400, 500]
        self.player_b_hitpoints = [300, 400, 500]
        self.current_state = 'WAITING'
        self.next_state = 'WAITING'

    def send_hitpoints(self, player_topic, hitpoints):
        rospy.loginfo(f"Sending hitpoints to {player_topic}: {hitpoints}")
        hitpoints_msg = ','.join(str(hp) for hp in hitpoints)
        rospy.Publisher(player_topic, String, queue_size=1).publish(hitpoints_msg)

    def receive_attack_moves(self, player_topic):
        rospy.loginfo(f"Waiting for attack moves from {player_topic}...")
        msg = rospy.wait_for_message(player_topic, String)
        attack_moves = msg.data.split(',')
        return attack_moves

    def update_hitpoints(self, player_hitpoints, opponent_attack_moves):
        updated_hitpoints = player_hitpoints.copy()
        for move in opponent_attack_moves:
            if move == '1':
                updated_hitpoints = [hp - 0.2 * max_hp for hp, max_hp in zip(updated_hitpoints, player_hitpoints)]
            elif move.startswith('2'):
                opponent_monster = move.split()[1]
                index = self.monster_names.index(opponent_monster)
                updated_hitpoints[index] -= 0.1 * player_hitpoints[index]
        return updated_hitpoints

    def server_callback(self, msg):
        hitpoints = msg.data.split(',')
        rospy.loginfo(f"Received hitpoints from the player: {hitpoints}")
        if self.current_state == 'WAITING':
            self.current_state = 'SENDING_HITPOINTS'

    def run(self):
        rospy.Subscriber(self.player_a_topic, String, self.server_callback)
        rospy.Subscriber(self.player_b_topic, String, self.server_callback)

        while not rospy.is_shutdown():
            if self.current_state == 'WAITING':
                rospy.sleep(1)
            elif self.current_state == 'SENDING_HITPOINTS':
                self.send_hitpoints(self.player_a_topic, self.player_a_hitpoints)
                self.send_hitpoints(self.player_b_topic, self.player_b_hitpoints)
                self.next_state = 'RECEIVING_MOVES'
                rospy.sleep(1)
                self.current_state = self.next_state
            elif self.current_state == 'RECEIVING_MOVES':
                player_a_moves = self.receive_attack_moves(self.player_a_topic)
                player_b_moves = self.receive_attack_moves(self.player_b_topic)

                self.player_a_hitpoints = self.update_hitpoints(self.player_a_hitpoints, player_b_moves)
                self.player_b_hitpoints = self.update_hitpoints(self.player_b_hitpoints, player_a_moves)

                rospy.loginfo("Player A attack moves: " + ", ".join(player_a_moves))
                rospy.loginfo("Player B attack moves: " + ", ".join(player_b_moves))

                self.send_hitpoints(self.player_a_topic, self.player_a_hitpoints)
                self.send_hitpoints(self.player_b_topic, self.player_b_hitpoints)

                rospy.loginfo("Updated Player A hitpoints: " + ', '.join(str(hp) for hp in self.player_a_hitpoints))
                rospy.loginfo("Updated Player B hitpoints: " + ', '.join(str(hp) for hp in self.player_b_hitpoints))

                if all(hp <= 0 for hp in self.player_a_hitpoints):
                    rospy.loginfo("Player B wins!")
                    break
                elif all(hp <= 0 for hp in self.player_b_hitpoints):
                    rospy.loginfo("Player A wins!")
                    break

                self.next_state = 'SENDING_HITPOINTS'
                rospy.sleep(1)
                self.current_state = self.next_state


if __name__ == '__main__':
    moderator = GameModerator()
    moderator.run()
