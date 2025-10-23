"""
ROS2 Service Server implementation using Zenoh queryables.

Based on rmw_zenoh design:
- Uses declare_queryable for service servers
- Service key expression: <domain_id>/<service_name>/<type_name>/<type_hash>
- Attachment contains: sequence_number, timestamp, client_gid
"""

from __future__ import annotations
import asyncio
import struct
import time
import logging
from typing import TYPE_CHECKING, Any, Callable, Optional, Type

if TYPE_CHECKING:
    from .node import Node

try:
    import zenoh
    ZENOH_AVAILABLE = True
except ImportError:
    ZENOH_AVAILABLE = False

logger = logging.getLogger(__name__)


class Service:
    """
    ROS2-compatible service server using Zenoh queryables.
    
    Example:
        from ros2_zenoh_python import Node
        from example_interfaces.srv import AddTwoInts
        
        async def handle_add(request):
            return AddTwoInts.Response(sum=request.a + request.b)
        
        async with Node('server') as node:
            srv = node.create_service(AddTwoInts, 'add_two_ints', handle_add)
            await node.spin()
    """
    
    def __init__(self, srv_type: Type, service_name: str, callback: Callable,
                 node: Optional[Node] = None, qos_profile: Optional[dict] = None):
        """
        Create a service server.
        
        Args:
            srv_type: Service type class (e.g., AddTwoInts)
            service_name: Name of the service (e.g., 'add_two_ints')
            callback: Callable that takes request and returns response
            node: Parent node (None for legacy standalone mode)
            qos_profile: QoS settings (currently unused for services)
        """
        if not ZENOH_AVAILABLE:
            raise RuntimeError("Zenoh is required. Install with: pip install eclipse-zenoh")
        
        self.srv_type = srv_type
        self.service_name = service_name
        self.callback = callback
        self.qos_profile = qos_profile or {}
        self.node = node
        
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
        
        # Check if callback is async
        self.is_async_callback = asyncio.iscoroutinefunction(callback)
        if self.is_async_callback:
            try:
                self.loop = asyncio.get_running_loop()
            except RuntimeError:
                self.loop = None
                logger.warning("Async callback but no running event loop - will run synchronously")
        
        # Declare queryable (without wildcards, as per rmw_zenoh spec)
        # Set complete=True to match rmw_zenoh behavior
        logger.debug(f"[SERVICE] Declaring queryable with key: {self.key_expr}")
        self.queryable = self.session.declare_queryable(
            self.key_expr,
            self._handle_query,
            complete=True
        )
        
        # Declare liveliness token for service discovery
        if node is not None:
            type_name = self.srv_type.Request.DDS_TYPE_NAME.replace('_Request_', '_')
            # Use SERVICE-level type hash for liveliness
            type_hash = self.srv_type.TYPE_HASH
            qos = self.qos_profile or {}
            self.liveliness_token = node.liveliness_manager.declare_service_server(
                self.service_name, type_name, type_hash, qos, node_name=node.node_name
            )
        
        logger.info(f"Service server created: {self.service_name}")
        logger.debug(f"  Key expression: {self.key_expr}")
    
    def _build_key_expr(self) -> str:
        """Build Zenoh key expression for service."""
        # Format: <domain_id>/<fully_qualified_name>/<type_name>/<type_hash>
        import os
        domain_id = os.environ.get('ROS_DOMAIN_ID', '0')
        
        # Get type name and hash from service type (NOT Request type)
        # Services use the base service type name (without _Request/_Response)
        type_name = self.srv_type.Request.DDS_TYPE_NAME.replace('_Request_', '_')
        # Use SERVICE-level type hash, not Request hash!
        type_hash = self.srv_type.TYPE_HASH
        
        # Service name already has leading / for absolute names
        service_name = self.service_name.lstrip('/')
        
        key_expr = f"{domain_id}/{service_name}/{type_name}/{type_hash}"
        return key_expr
    
    def _handle_query(self, query: zenoh.Query):
        """Handle incoming service request."""
        logger.debug(f"[SERVICE] Received query on: {query.key_expr}")
        try:
            # Parse attachment (sequence number, timestamp, client GID)
            attachment = self._parse_attachment(query)
            
            # Deserialize request
            payload = bytes(query.payload)
            request = self.srv_type.Request.deserialize(payload)
            
            logger.debug(f"Service request received: {request}")
            
            # Call user callback
            if self.is_async_callback and self.loop is not None:
                # For async callbacks, schedule the handler to execute in the event loop
                # and send the reply asynchronously
                
                async def async_reply_wrapper():
                    """Execute async handler and send reply"""
                    try:
                        response = await self.callback(request)
                        
                        # Serialize response
                        response_payload = response.serialize()
                        
                        # Build response attachment
                        response_attachment = self._build_attachment(
                            attachment['sequence_number'],
                            attachment['client_gid']
                        )
                        
                        # Send reply
                        query.reply(
                            self.key_expr,
                            response_payload,
                            attachment=response_attachment
                        )
                        logger.debug("Async handler completed and replied")
                        
                    except Exception as e:
                        logger.error(f"Async handler failed: {e}", exc_info=True)
                        try:
                            query.reply_err(f"Service error: {e}")
                        except:
                            pass
                
                # Schedule the async handler - don't wait for it
                asyncio.run_coroutine_threadsafe(async_reply_wrapper(), self.loop)
                
                # Return immediately - the reply will be sent asynchronously
                return
            else:
                # Sync callback - process immediately
                response = self.callback(request)
            
            # Serialize response
            response_payload = response.serialize()
            
            # Build response attachment (echo back sequence number and client GID)
            response_attachment = self._build_attachment(
                attachment['sequence_number'],
                attachment['client_gid']
            )
            
            # Send reply
            query.reply(
                query.key_expr,
                response_payload,
                attachment=response_attachment
            )
            
            logger.debug(f"Service response sent: {response}")
            
        except Exception as e:
            logger.error(f"Error handling service request: {e}", exc_info=True)
    
    def _parse_attachment(self, query: zenoh.Query) -> dict:
        """
        Parse attachment from query.
        
        Attachment format (matches rmw_zenoh zenoh::ext::Serializer):
        - 8 bytes: sequence number (int64, little-endian)
        - 8 bytes: timestamp (int64, little-endian)
        - 1 byte: VarInt length prefix (0x10 = 16 in LEB128)
        - 16 bytes: client GID
        """
        if query.attachment is None:
            logger.warning("No attachment in query")
            return {
                'sequence_number': 0,
                'timestamp': 0,
                'client_gid': b'\x00' * 16
            }
        
        attachment_bytes = bytes(query.attachment)
        
        if len(attachment_bytes) < 33:  # 8 + 8 + 1 + 16
            logger.warning(f"Attachment too short: {len(attachment_bytes)} bytes (expected 33)")
            return {
                'sequence_number': 0,
                'timestamp': 0,
                'client_gid': b'\x00' * 16
            }
        
        sequence_number = struct.unpack('<q', attachment_bytes[0:8])[0]
        timestamp = struct.unpack('<q', attachment_bytes[8:16])[0]
        gid_len_varint = attachment_bytes[16]  # Should be 0x10 (16 in VarInt/LEB128)
        client_gid = attachment_bytes[17:33]  # 16 bytes after the length prefix
        
        
        return {
            'sequence_number': sequence_number,
            'timestamp': timestamp,
            'client_gid': client_gid
        }
    
    def _build_attachment(self, sequence_number: int, client_gid: bytes) -> bytes:
        """
        Build attachment for response.
        
        Attachment format (matches rmw_zenoh zenoh::ext::Serializer):
        - 8 bytes: sequence number (same as request)
        - 8 bytes: timestamp (current time)
        - 1 byte: VarInt length prefix (0x10 = 16 in LEB128)
        - 16 bytes: client GID (same as request)
        """
        timestamp = int(time.time() * 1e9)  # nanoseconds since epoch
        
        attachment = struct.pack('<q', sequence_number)  # sequence number
        attachment += struct.pack('<q', timestamp)  # timestamp
        attachment += b'\x10'  # VarInt(16) - length prefix for GID array
        attachment += client_gid  # client GID (16 bytes)
        
        return attachment
    
    async def adestroy(self):
        """Async cleanup."""
        self.destroy()  # Cleanup is already non-blocking, reuse sync version
    
    def destroy(self):
        """Sync cleanup."""
        if hasattr(self, '_destroyed') and self._destroyed:
            return  # Already destroyed
        
        if hasattr(self, 'queryable'):
            try:
                self.queryable.undeclare()
            except Exception as e:
                logger.debug(f"Error undeclaring queryable: {e}")
        if hasattr(self, 'liveliness_token'):
            try:
                self.liveliness_token.undeclare()
            except Exception as e:
                logger.debug(f"Error undeclaring liveliness token: {e}")
        if hasattr(self, '_owns_session') and self._owns_session:
            self.session.close()
        
        self._destroyed = True
        logger.debug(f"Service server destroyed: {self.service_name}")



