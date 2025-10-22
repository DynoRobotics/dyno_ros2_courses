"""
ROS2 Service Client implementation using Zenoh queries.

Based on rmw_zenoh design:
- Uses get (query) operation for service calls
- Service key expression: <domain_id>/<service_name>/<type_name>/<type_hash>
- Attachment contains: sequence_number, timestamp, client_gid
"""

from __future__ import annotations
import asyncio
import struct
import time
import uuid
import logging
from typing import TYPE_CHECKING, Any, Optional, Type

if TYPE_CHECKING:
    from .node import Node

try:
    import zenoh
    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False

logger = logging.getLogger(__name__)


class Client:
    """
    ROS2-compatible service client using Zenoh queries.
    
    Example:
        from ros2_zenoh_python import Node
        from example_interfaces.srv import AddTwoInts
        
        async with Node('client') as node:
            client = node.create_client(AddTwoInts, 'add_two_ints')
            request = AddTwoInts.Request(a=5, b=7)
            response = await client.call_async(request)
            print(f"Result: {response.sum}")
    """
    
    def __init__(self, srv_type: Type, service_name: str,
                 node: Optional[Node] = None, qos_profile: Optional[dict] = None):
        """
        Create a service client.
        
        Args:
            srv_type: Service type class (e.g., AddTwoInts)
            service_name: Name of the service (e.g., 'add_two_ints')
            node: Parent node (None for legacy standalone mode)
            qos_profile: QoS settings (currently unused for services)
        """
        if not ZENOH_AVAILABLE:
            raise RuntimeError("Zenoh is required. Install with: pip install eclipse-zenoh")
        
        self.srv_type = srv_type
        self.service_name = service_name
        self.qos_profile = qos_profile or {}
        self.node = node
        self._sequence_number = 0
        self._gid = uuid.uuid4().bytes  # 16 bytes
        
        if node is not None:
            self.session = node.session
            self.namespace = node.namespace
        else:
            # Legacy mode: create own session
            config = zenoh.Config()
            config.insert_json5('mode', '"client"')
            config.insert_json5('connect/endpoints', '["tcp/localhost:7447"]')
            self.session = zenoh.open(config)
            self.namespace = ""
            self._owns_session = True
        
        # Resolve service name with namespace
        from .name_utils import resolve_topic_name
        self.service_name = resolve_topic_name(service_name, self.namespace)
        logger.debug(f"Resolved service: '{service_name}' -> '{self.service_name}' (namespace: '{self.namespace}')")
        
        # Build Zenoh key expression
        self.key_expr = self._build_key_expr()
        
        # Declare liveliness token for service discovery
        if node is not None:
            type_name = self.srv_type.Request.DDS_TYPE_NAME.replace('_Request_', '_')
            # Use SERVICE-level type hash for liveliness
            type_hash = self.srv_type.TYPE_HASH
            qos = self.qos_profile or {}
            self.liveliness_token = node.liveliness_manager.declare_service_client(
                self.service_name, type_name, type_hash, qos, node_name=node.node_name
            )
        
        logger.info(f"Service client created: {self.service_name}")
        logger.debug(f"  Key expression: {self.key_expr}")
    
    def _build_key_expr(self) -> str:
        """Build Zenoh key expression for service."""
        import os
        domain_id = os.environ.get('ROS_DOMAIN_ID', '0')
        
        # Get type name from Request DDS name, but use SERVICE-level hash
        type_name = self.srv_type.Request.DDS_TYPE_NAME.replace('_Request_', '_')
        # Use SERVICE-level type hash, not Request hash!
        type_hash = self.srv_type.TYPE_HASH
        
        service_name = self.service_name.lstrip('/')
        
        key_expr = f"{domain_id}/{service_name}/{type_name}/{type_hash}"
        return key_expr
    
    async def call_async(self, request: Any, timeout: float = 5.0) -> Any:
        """
        Call service asynchronously.
        
        Args:
            request: Service request message
            timeout: Timeout in seconds (default: 5.0)
            
        Returns:
            Service response message
            
        Raises:
            TimeoutError: If service call times out
            RuntimeError: If service call fails
        """
        # Increment sequence number
        self._sequence_number += 1
        
        # Serialize request
        payload = request.serialize()
        
        # Build attachment
        attachment = self._build_attachment(self._sequence_number)
        
        logger.debug(f"[CLIENT] Sending service request to: {self.key_expr}")
        logger.debug(f"Sending service request: {request}")
        
        # Run the blocking Zenoh get() in a thread executor so we don't block the event loop
        loop = asyncio.get_event_loop()
        
        def blocking_get():
            """Execute blocking Zenoh get() in thread"""
            replies = self.session.get(
                self.key_expr,
                payload=payload,
                attachment=attachment,
                timeout=timeout
            )
            
            # Get first reply
            for reply in replies:
                logger.debug(f"[CLIENT] Got reply: {reply}")
                if hasattr(reply, 'ok') and reply.ok:
                    # Deserialize response
                    response_payload = bytes(reply.ok.payload)
                    response = self.srv_type.Response.deserialize(response_payload)
                    logger.debug(f"Received service response: {response}")
                    return response
            
            # No response received
            return None
        
        logger.debug(f"[CLIENT] Waiting for replies...")
        
        # Run in thread executor to avoid blocking event loop
        # Use None (default executor) to avoid thread pool exhaustion issues
        response = await loop.run_in_executor(None, blocking_get)
        
        if response is None:
            logger.error(f"Service call failed: No response received from {self.service_name}")
            raise TimeoutError(f"Service call to {self.service_name} timed out after {timeout}s")
        
        return response
    
    def _build_attachment(self, sequence_number: int) -> bytes:
        """
        Build attachment for request.
        
        Attachment format (matches rmw_zenoh zenoh::ext::Serializer):
        - 8 bytes: sequence number (int64, little-endian)
        - 8 bytes: timestamp (int64, little-endian)
        - 1 byte: VarInt length prefix (0x10 = 16 in LEB128)
        - 16 bytes: client GID
        """
        timestamp = int(time.time() * 1e9)  # nanoseconds since epoch
        
        attachment = struct.pack('<q', sequence_number)  # sequence number
        attachment += struct.pack('<q', timestamp)  # timestamp
        attachment += b'\x10'  # VarInt(16) - length prefix for GID array
        attachment += self._gid  # client GID (16 bytes)
        
        return attachment
    
    async def wait_for_server(self, timeout: float = 5.0) -> bool:
        """
        Wait for service server to be available.
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            True if server is found, False if timeout
        """
        import os
        domain_id = os.environ.get('ROS_DOMAIN_ID', '0')
        service_name = self.service_name.lstrip('/')
        
        # Build liveliness pattern to match service servers
        # Actual format: @ros2_lv/{domain}/{participant}/{entity_type}/{entity_id}/SS/{ns}/{ns2}/{node}/{service_encoded}/{type}/{hash}/{qos}
        # Use proper Zenoh key expression syntax:
        # - * matches a single segment
        # - ** matches zero or more segments (must be complete segment)
        # Service names are encoded as: %{name with / replaced by %}
        service_name_encoded = '%' + service_name.replace('/', '%')
        # Match: domain/participant/entity_type/entity_id/SS/ns/ns2/node/service_name/...rest
        pattern = f"@ros2_lv/{domain_id}/*/*/*/SS/*/*/*/{service_name_encoded}/**"
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            # Query liveliness
            replies = self.session.liveliness().get(pattern, timeout=0.5)
            
            # Check if any server matches
            for reply in replies:
                if reply.ok:
                    # Found a server
                    logger.debug(f"Found server for {self.service_name}: {reply.ok.key_expr}")
                    return True
            
            # Wait a bit before retrying
            await asyncio.sleep(0.1)
        
        logger.warning(f"Service server {self.service_name} not found after {timeout}s")
        return False
    
    async def adestroy(self):
        """Async cleanup."""
        if hasattr(self, 'liveliness_token'):
            self.liveliness_token.undeclare()
        if hasattr(self, '_owns_session') and self._owns_session:
            self.session.close()
        logger.debug(f"Service client destroyed: {self.service_name}")
    
    def destroy(self):
        """Sync cleanup."""
        if hasattr(self, 'liveliness_token'):
            self.liveliness_token.undeclare()
        if hasattr(self, '_owns_session') and self._owns_session:
            self.session.close()
        logger.debug(f"Service client destroyed: {self.service_name}")



