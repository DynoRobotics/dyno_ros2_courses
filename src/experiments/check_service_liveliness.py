#!/usr/bin/env python3
"""
Check for ROS2 service liveliness tokens with the correct @ros2_lv prefix.
Based on rmw_zenoh design documentation.
"""

import zenoh


def main():
    config = zenoh.Config()
    config.insert_json5('mode', '"client"')
    config.insert_json5('connect/endpoints', '["tcp/localhost:7447"]')
    
    print("🔗 Connecting to Zenoh...")
    session = zenoh.open(config)
    
    # According to rmw_zenoh docs, liveliness tokens use @ros2_lv prefix
    # Format: @ros2_lv/<domain_id>/<session_id>/<node_id>/<entity_id>/<entity_kind>/...
    
    patterns = [
        "@ros2_lv/**",           # All ROS2 liveliness tokens
        "@ros2_lv/**/SS/**",     # Service Servers
        "@ros2_lv/**/SC/**",     # Service Clients
        "@ros2_lv/**/MP/**",     # Message Publishers
        "@ros2_lv/**/MS/**",     # Message Subscribers
    ]
    
    for pattern in patterns:
        print(f"\n🔍 Pattern: {pattern}")
        print("=" * 70)
        
        try:
            replies = session.liveliness().get(pattern)
            count = 0
            
            for reply in replies:
                count += 1
                reply_str = str(reply)
                
                if 'key_expr:' in reply_str:
                    import re
                    match = re.search(r"key_expr:\s*ke`([^`]+)`", reply_str)
                    if match:
                        key = match.group(1)
                        
                        # Parse entity kind
                        if '/SS/' in key:
                            entity = "🔧 SERVICE SERVER"
                        elif '/SC/' in key:
                            entity = "🔌 SERVICE CLIENT"
                        elif '/MP/' in key:
                            entity = "📤 PUBLISHER"
                        elif '/MS/' in key:
                            entity = "📥 SUBSCRIBER"
                        elif '/NN/' in key:
                            entity = "🤖 NODE"
                        else:
                            entity = "❓ UNKNOWN"
                        
                        print(f"   {entity}")
                        print(f"      {key}")
            
            if count == 0:
                print("   (no tokens found)")
            else:
                print(f"\n   Total: {count} tokens")
                
        except Exception as e:
            print(f"   Error: {e}")
    
    session.close()
    print("\n✅ Done")


if __name__ == '__main__':
    main()

