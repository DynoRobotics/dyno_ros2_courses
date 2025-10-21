#!/usr/bin/env python3
"""
Introspect what topics are created when a service is active.
Check for patterns like rq/ (request) and rs/ (response).
"""

import rclpy
from rclpy.node import Node as RclpyNode
import time


def main():
    rclpy.init()
    node = RclpyNode('introspector')
    
    print("📋 Topics BEFORE service creation:")
    print("=" * 60)
    topics_before = node.get_topic_names_and_types()
    for topic, types in sorted(topics_before):
        if 'add' in topic.lower() or 'rq' in topic or 'rs' in topic:
            print(f"  {topic}")
            for t in types:
                print(f"    Type: {t}")
    
    print("\n🔧 Creating service...")
    from example_interfaces.srv import AddTwoInts
    
    def service_callback(request, response):
        response.sum = request.a + request.b
        return response
    
    srv = node.create_service(AddTwoInts, 'test_service', service_callback)
    time.sleep(0.5)  # Let it settle
    
    print("\n📋 Topics AFTER service creation:")
    print("=" * 60)
    topics_after = node.get_topic_names_and_types()
    for topic, types in sorted(topics_after):
        if 'test_service' in topic.lower() or 'add' in topic.lower() or 'rq' in topic or 'rs' in topic or 'reply' in topic or 'request' in topic:
            print(f"  {topic}")
            for t in types:
                print(f"    Type: {t}")
    
    print("\n🔍 NEW topics (that weren't there before):")
    print("=" * 60)
    new_topics = set(t for t, _ in topics_after) - set(t for t, _ in topics_before)
    for topic in sorted(new_topics):
        if 'test_service' in topic.lower() or 'rq' in topic or 'rs' in topic:
            types = dict(topics_after)[topic]
            print(f"  {topic}")
            for t in types:
                print(f"    Type: {t}")
    
    print("\n✅ Done. Service is active for 5 seconds...")
    time.sleep(5)
    
    node.destroy_service(srv)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


