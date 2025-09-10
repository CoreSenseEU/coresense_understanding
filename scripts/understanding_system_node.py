#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

import std_msgs.msg
import coresense_msgs.msg
import coresense_msgs.action import QueryReasoner
from coresense_msgs.srv import AddKnowledge, PopKnowledge, ListKnowledge


class UnderstandingSystemNode(Node):
    def __init__(self):
        super().__init__('UnderstandingSystemNode')

        self.add_knowledge_client = self.create_client(AddKnowledge, '/add_knowledge')
        self.pop_knowledge_client = self.create_client(PopKnowledge, '/pop_knowledge')
        self.list_knowledge_client = self.create_client(ListKnowledge, '/list_knowledge')

        self.query_reasoner_client = ActionClient(self, QueryReasoner, '/query_reasoner')

        while not self.add_knowledge_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddKnowledge.Request()


    # TODO create an understanding action server
    # TODO create the logic reading algorithm and datastructure
    # TODO create the result to BT xml converter
    # TODO create the ros-system-analysis algorithm that reads semantic annotation and creates the respective logic representation
    
    def add_knowledge(self, label, theory):
        self.req.tptp = theory
        self.req.label = label
        self.future = self.add_knowledge_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def query_reasoner(self, query, config):
        goal_msg = QueryReasoner.Goal()
        goal_msg.query = ''
        goal_msg.config = ''
        self.query_reasoner_client.wait_for_server()
        return self.query_reasoner_client.send_goal_async(goal_msg)



def main(args=None):
    rclpy.init(args=args)
    node = UnderstandingSystemNode()

    # send service request
    response = node.send_request(label, theory)

    # send action request
    future = node.query_reasoner(query, config)

    # wait for action to return
    rclpy.spin_until_future_complete(node, future)

    node.get_logger().info('this is how to log messages')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
