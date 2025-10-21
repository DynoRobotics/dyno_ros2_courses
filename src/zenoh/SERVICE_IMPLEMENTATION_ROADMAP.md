# ROS2 Service Support Implementation Roadmap

## ✅ What We Discovered

### Introspection Results

1. **Services DO use Zenoh liveliness tokens** with the `@ros2_lv` prefix
2. **Example service token** (`/add_two_ints`):
   ```
   @ros2_lv/0/f1eec80b0794deb18ff9ee5a0182029d/0/10/SS/%/%/simple_service_server/%add_two_ints/example_interfaces::srv::dds_::AddTwoInts_/RIHS01_e118de6bf5eeb66a2491b5bda11202e7b68f198d6f67922cf30364858239c81a/::,10:,:,:,,
   ```

3. **Token structure**:
   - Entity type: `SS` (Service Server) or `SC` (Service Client)
   - Service name is mangled: `/` → `%` (e.g., `%add_two_ints`)
   - Includes type hash (RIHS01)
   - Includes QoS

4. **Service implementation** (from rmw_zenoh docs):
   - **Server**: Uses `declare_queryable` with complete queryable
   - **Client**: Uses `get` operation (query)
   - **Key expression**: `<domain_id>/<fully_qualified_name>/<type_name>/<type_hash>`
   - **Attachment**: Contains sequence number, timestamp, client/server GID

### Key Expression for AddTwoInts Service

```
0/add_two_ints/example_interfaces::srv::dds_::AddTwoInts_/RIHS01_e118de6bf5eeb66a2491b5bda11202e7b68f198d6f67922cf30364858239c81a
```

## 📋 Implementation Tasks

### Task 1: Service Type Generation ⏳ IN PROGRESS

**Status**: Parser added, need to wire into generation pipeline

**What's Done**:
- ✅ Added `ServiceInfo` dataclass to `generator.py`
- ✅ Added `_parse_srv_file()` method to parse `.srv` files

**What's Left**:
1. Update `_discover_messages()` to also discover `.srv` files
2. Store services in `Generator` (add `services_by_package` dict)
3. Add service directory discovery in `_discover_specific_packages()`
4. Create Jinja2 template for services (`service.py.jinja2`)
5. Update `PythonGenerator` to generate service files
6. Compute hashes for Request/Response messages
7. Test generation with `example_interfaces/srv/AddTwoInts`

**Service File Structure** (example_interfaces/srv/add_two_ints.py):
```python
from dataclasses import dataclass
from pycdr2 import IdlStruct
from .._encodings import serialize_cdr, deserialize_cdr, serialize_json, deserialize_json
from functools import partial

@dataclass
class AddTwoInts_Request(IdlStruct):
    a: int64
    b: int64
    TYPE_HASH = "RIHS01_..."
    DDS_TYPE_NAME = "example_interfaces::srv::dds_::AddTwoInts_Request_"
    
    # Encoding methods...
    @classmethod
    def get_serializer(cls, encoding='cdr'):
        ...
    
    @classmethod
    def get_deserializer(cls, encoding='cdr'):
        ...

@dataclass
class AddTwoInts_Response(IdlStruct):
    sum: int64
    TYPE_HASH = "RIHS01_..."
    DDS_TYPE_NAME = "example_interfaces::srv::dds_::AddTwoInts_Response_"
    
    # Encoding methods...

class AddTwoInts:
    Request = AddTwoInts_Request
    Response = AddTwoInts_Response
    TYPE_HASH = "RIHS01_..."  # Service-level hash
    DDS_TYPE_NAME = "example_interfaces::srv::dds_::AddTwoInts_"
```

### Task 2: Service Class (Server)

**File**: `ros2_zenoh_python/service.py`

**Implementation**:
```python
from __future__ import annotations
import asyncio
import struct
import time
import uuid
from typing import TYPE_CHECKING, Any, Callable, Optional, Type
import zenoh
import logging

if TYPE_CHECKING:
    from .node import Node

logger = logging.getLogger(__name__)


class Service:
    """ROS2-compatible service server using Zenoh queryables."""
    
    def __init__(self, srv_type: Type, service_name: str, callback: Callable,
                 node: Optional[Node] = None, qos_profile: Optional[dict] = None):
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
        
        # Resolve service name with namespace
        from .name_utils import resolve_topic_name
        self.service_name = resolve_topic_name(service_name, self.namespace)
        logger.debug(f"Resolved service: '{service_name}' -> '{self.service_name}' (namespace: '{self.namespace}')")
        
        # Build Zenoh key expression
        self.key_expr = self._build_key_expr()
        
        # Check if callback is async
        self.is_async_callback = asyncio.iscoroutinefunction(callback)
        if self.is_async_callback:
            self.loop = asyncio.get_running_loop()
        
        # Declare queryable
        self.queryable = self.session.declare_queryable(
            self.key_expr,
            self._handle_query
        )
        
        # Declare liveliness token (SS = Service Server)
        if node is not None:
            self._declare_liveliness_token()
        
        logger.debug(f"Service server created: {self.service_name}")
    
    def _build_key_expr(self) -> str:
        """Build Zenoh key expression for service."""
        # Format: <domain_id>/<fully_qualified_name>/<type_name>/<type_hash>
        domain_id = 0  # TODO: Get from ROS_DOMAIN_ID env var
        type_name = self.srv_type.DDS_TYPE_NAME
        type_hash = self.srv_type.TYPE_HASH
        
        # Service name already has leading / for absolute names
        service_name = self.service_name.lstrip('/')
        
        key_expr = f"{domain_id}/{service_name}/{type_name}/{type_hash}"
        logger.debug(f"Service key expression: {key_expr}")
        return key_expr
    
    def _declare_liveliness_token(self):
        """Declare liveliness token for service discovery."""
        # Format: @ros2_lv/<domain>/<session_id>/<node_id>/<entity_id>/SS/<enclave>/<namespace>/<node_name>/<service_name>/<type_name>/<type_hash>/<qos>
        # TODO: Implement proper liveliness tokens
        pass
    
    def _handle_query(self, query: zenoh.Query):
        """Handle incoming service request."""
        try:
            # Parse attachment (sequence number, timestamp, client GID)
            attachment = self._parse_attachment(query)
            
            # Deserialize request
            payload = bytes(query.payload())
            request = self.srv_type.Request.deserialize(payload)
            
            logger.debug(f"Service request received: {request}")
            
            # Call user callback
            if self.is_async_callback:
                # Schedule async callback
                future = asyncio.run_coroutine_threadsafe(
                    self.callback(request),
                    self.loop
                )
                response = future.result()
            else:
                response = self.callback(request)
            
            # Serialize response
            response_payload = response.serialize()
            
            # Build response attachment
            response_attachment = self._build_attachment(attachment['sequence_number'], attachment['client_gid'])
            
            # Send reply
            query.reply(zenoh.Sample(query.key_expr(), response_payload, attachment=response_attachment))
            
            logger.debug(f"Service response sent: {response}")
            
        except Exception as e:
            logger.error(f"Error handling service request: {e}", exc_info=True)
    
    def _parse_attachment(self, query: zenoh.Query) -> dict:
        """Parse attachment from query."""
        if not query.attachment():
            return {'sequence_number': 0, 'timestamp': 0, 'client_gid': b'\x00' * 16}
        
        attachment_bytes = bytes(query.attachment())
        
        # Attachment format (version 3):
        # - 8 bytes: sequence number (int64, little-endian)
        # - 8 bytes: timestamp (int64, little-endian)
        # - 1 byte: GID length (always 16)
        # - 16 bytes: client GID
        
        if len(attachment_bytes) < 33:
            return {'sequence_number': 0, 'timestamp': 0, 'client_gid': b'\x00' * 16}
        
        sequence_number = struct.unpack('<q', attachment_bytes[0:8])[0]
        timestamp = struct.unpack('<q', attachment_bytes[8:16])[0]
        gid_length = attachment_bytes[16]
        client_gid = attachment_bytes[17:17+gid_length]
        
        return {
            'sequence_number': sequence_number,
            'timestamp': timestamp,
            'client_gid': client_gid
        }
    
    def _build_attachment(self, sequence_number: int, client_gid: bytes) -> bytes:
        """Build attachment for response."""
        # Attachment format (version 3):
        # - 8 bytes: sequence number (same as request)
        # - 8 bytes: timestamp (current time)
        # - 1 byte: GID length (always 16)
        # - 16 bytes: client GID (same as request)
        
        timestamp = int(time.time() * 1e9)  # nanoseconds since epoch
        
        attachment = struct.pack('<q', sequence_number)  # sequence number
        attachment += struct.pack('<q', timestamp)  # timestamp
        attachment += struct.pack('<B', len(client_gid))  # GID length
        attachment += client_gid  # client GID
        
        return attachment
    
    async def adestroy(self):
        """Async cleanup."""
        if hasattr(self, 'queryable'):
            self.queryable.undeclare()
        logger.debug(f"Service server destroyed: {self.service_name}")
    
    def destroy(self):
        """Sync cleanup."""
        if hasattr(self, 'queryable'):
            self.queryable.undeclare()
        logger.debug(f"Service server destroyed: {self.service_name}")
```

### Task 3: Client Class

**File**: `ros2_zenoh_python/client.py`

**Implementation**:
```python
from __future__ import annotations
import asyncio
import struct
import time
import uuid
from typing import TYPE_CHECKING, Any, Optional, Type
import zenoh
import logging

if TYPE_CHECKING:
    from .node import Node

logger = logging.getLogger(__name__)


class Client:
    """ROS2-compatible service client using Zenoh queries."""
    
    def __init__(self, srv_type: Type, service_name: str,
                 node: Optional[Node] = None, qos_profile: Optional[dict] = None):
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
        
        # Resolve service name with namespace
        from .name_utils import resolve_topic_name
        self.service_name = resolve_topic_name(service_name, self.namespace)
        logger.debug(f"Resolved service: '{service_name}' -> '{self.service_name}' (namespace: '{self.namespace}')")
        
        # Build Zenoh key expression
        self.key_expr = self._build_key_expr()
        
        # Declare liveliness token (SC = Service Client)
        if node is not None:
            self._declare_liveliness_token()
        
        logger.debug(f"Service client created: {self.service_name}")
    
    def _build_key_expr(self) -> str:
        """Build Zenoh key expression for service."""
        domain_id = 0  # TODO: Get from ROS_DOMAIN_ID env var
        type_name = self.srv_type.DDS_TYPE_NAME
        type_hash = self.srv_type.TYPE_HASH
        
        service_name = self.service_name.lstrip('/')
        
        key_expr = f"{domain_id}/{service_name}/{type_name}/{type_hash}"
        logger.debug(f"Service key expression: {key_expr}")
        return key_expr
    
    def _declare_liveliness_token(self):
        """Declare liveliness token for service discovery."""
        # Format: @ros2_lv/.../SC/...
        # TODO: Implement proper liveliness tokens
        pass
    
    async def call_async(self, request: Any) -> Any:
        """Call service asynchronously."""
        # Increment sequence number
        self._sequence_number += 1
        
        # Serialize request
        payload = request.serialize()
        
        # Build attachment
        attachment = self._build_attachment(self._sequence_number)
        
        logger.debug(f"Sending service request: {request}")
        
        # Send query (target=ALL_COMPLETE for complete queryables)
        replies = self.session.get(
            self.key_expr,
            payload=payload,
            attachment=attachment,
            timeout=5.0  # 5 second timeout
        )
        
        # Get first reply
        for reply in replies:
            if hasattr(reply, 'ok') and reply.ok:
                # Deserialize response
                response_payload = bytes(reply.ok.payload())
                response = self.srv_type.Response.deserialize(response_payload)
                logger.debug(f"Received service response: {response}")
                return response
        
        logger.error(f"Service call failed: No response received")
        raise TimeoutError(f"Service call to {self.service_name} timed out")
    
    def _build_attachment(self, sequence_number: int) -> bytes:
        """Build attachment for request."""
        # Attachment format (version 3):
        # - 8 bytes: sequence number
        # - 8 bytes: timestamp
        # - 1 byte: GID length (always 16)
        # - 16 bytes: client GID
        
        timestamp = int(time.time() * 1e9)  # nanoseconds since epoch
        
        attachment = struct.pack('<q', sequence_number)  # sequence number
        attachment += struct.pack('<q', timestamp)  # timestamp
        attachment += struct.pack('<B', len(self._gid))  # GID length
        attachment += self._gid  # client GID
        
        return attachment
    
    async def adestroy(self):
        """Async cleanup."""
        logger.debug(f"Service client destroyed: {self.service_name}")
    
    def destroy(self):
        """Sync cleanup."""
        logger.debug(f"Service client destroyed: {self.service_name}")
```

### Task 4: Node Integration

**Changes to** `ros2_zenoh_python/node.py`:

```python
# Add to Node class:

def create_service(self, srv_type: Type, service_name: str, callback: Callable,
                  qos_profile: Optional[dict] = None) -> Service:
    """Create a service server."""
    from .service import Service
    
    srv = Service(
        srv_type,
        service_name,
        callback,
        node=self,
        qos_profile=qos_profile
    )
    self.services[service_name] = srv
    logger.debug(f"Created service server for '{service_name}'")
    return srv

def create_client(self, srv_type: Type, service_name: str,
                 qos_profile: Optional[dict] = None) -> Client:
    """Create a service client."""
    from .client import Client
    
    client = Client(
        srv_type,
        service_name,
        node=self,
        qos_profile=qos_profile
    )
    self.clients[service_name] = client
    logger.debug(f"Created service client for '{service_name}'")
    return client

# Add to __init__:
self.services = {}
self.clients = {}

# Add to adestroy_node:
for srv in self.services.values():
    await srv.adestroy()
for client in self.clients.values():
    await client.adestroy()
```

### Task 5: Examples

**File**: `examples/service_server_example.py`:
```python
#!/usr/bin/env python3
import asyncio
import logging
from ros2_zenoh_python import Node
from example_interfaces.srv import AddTwoInts

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


async def main():
    async with Node('add_two_ints_server') as node:
        def handle_request(request):
            result = request.a + request.b
            logger.info(f"Request: {request.a} + {request.b} = {result}")
            return AddTwoInts.Response(sum=result)
        
        srv = node.create_service(AddTwoInts, 'add_two_ints', handle_request)
        logger.info("Service ready!")
        
        await node.spin()


if __name__ == '__main__':
    asyncio.run(main())
```

**File**: `examples/service_client_example.py`:
```python
#!/usr/bin/env python3
import asyncio
import logging
from ros2_zenoh_python import Node
from example_interfaces.srv import AddTwoInts

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


async def main():
    async with Node('add_two_ints_client') as node:
        client = node.create_client(AddTwoInts, 'add_two_ints')
        
        # Make a few requests
        for i in range(5):
            request = AddTwoInts.Request(a=i, b=i*10)
            logger.info(f"Sending request: {request.a} + {request.b}")
            
            response = await client.call_async(request)
            logger.info(f"Response: {response.sum}")
            
            await asyncio.sleep(1)


if __name__ == '__main__':
    asyncio.run(main())
```

### Task 6: Tests

**File**: `tests/test_service.py`:
```python
import pytest
import asyncio
from ros2_zenoh_python import Node
from example_interfaces.srv import AddTwoInts


@pytest.mark.asyncio
async def test_service_basic(zenoh_session):
    """Test basic service call."""
    # Server callback
    def handle_add(request):
        return AddTwoInts.Response(sum=request.a + request.b)
    
    # Create server and client
    async with Node('server', zenoh_session=zenoh_session, enable_rosout=False) as server_node:
        async with Node('client', zenoh_session=zenoh_session, enable_rosout=False) as client_node:
            srv = server_node.create_service(AddTwoInts, '/test_add', handle_add)
            client = client_node.create_client(AddTwoInts, '/test_add')
            
            # Allow discovery
            await asyncio.sleep(0.1)
            
            # Call service
            request = AddTwoInts.Request(a=5, b=7)
            response = await client.call_async(request)
            
            assert response.sum == 12


@pytest.mark.asyncio
async def test_service_async_callback(zenoh_session):
    """Test service with async callback."""
    async def handle_add_async(request):
        await asyncio.sleep(0.01)  # Simulate async work
        return AddTwoInts.Response(sum=request.a + request.b)
    
    async with Node('server', zenoh_session=zenoh_session, enable_rosout=False) as server_node:
        async with Node('client', zenoh_session=zenoh_session, enable_rosout=False) as client_node:
            srv = server_node.create_service(AddTwoInts, '/test_add_async', handle_add_async)
            client = client_node.create_client(AddTwoInts, '/test_add_async')
            
            await asyncio.sleep(0.1)
            
            request = AddTwoInts.Request(a=10, b=20)
            response = await client.call_async(request)
            
            assert response.sum == 30


@pytest.mark.interop
@pytest.mark.asyncio
async def test_service_interop_with_rclpy(zenoh_session_client):
    """Test interop with rclpy service."""
    try:
        import rclpy
        from example_interfaces.srv import AddTwoInts as RclpyAddTwoInts
    except ImportError:
        pytest.skip("rclpy not available")
    
    # TODO: Implement interop test
```

## 🎯 Next Steps

1. Complete Task 1 (service generation) - **Current Focus**
2. Implement Service class (Task 2)
3. Implement Client class (Task 3)
4. Integrate with Node (Task 4)
5. Create examples (Task 5)
6. Add tests (Task 6)

## ⚠️ Open Questions

1. **Domain ID**: Should we read `ROS_DOMAIN_ID` env var? Default to 0?
2. **Liveliness tokens**: Full implementation needed for proper discovery
3. **QoS**: What QoS settings apply to services?
4. **Error handling**: How to handle service not available?
5. **Service hashing**: How to compute service-level TYPE_HASH (vs Request/Response hashes)?

## 📚 References

- [rmw_zenoh Design Doc](https://github.com/ros2/rmw_zenoh/blob/jazzy/docs/design.md)
- [Zenoh Query/Reply](https://zenoh.io/docs/manual/abstractions/#queryreply)
- [ROS2 Services](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)

