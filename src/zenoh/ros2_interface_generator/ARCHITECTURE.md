# ROS2 Interface Generator - Architecture

## System Architecture

```mermaid
flowchart TB
    subgraph Core["Generator Core"]
        Discovery["Message Discovery<br/>(File-based)"]
        Parser["Message Parser<br/>(fields, types, deps)"]
        TypeHash["Type Hash<br/>Computation<br/>(RIHS01)"]
    end
    
    subgraph Lang["Language Backends"]
        Python["Python Generator<br/>(dataclasses + templates)"]
        Rust["Rust Generator<br/>(future)"]
        TS["TypeScript Generator<br/>(future)"]
        C["C/C++ Generator<br/>(future)"]
    end
    
    subgraph Enc["Encoding Backends"]
        CDR["CDR Encoding<br/>(pycdr2)<br/>✅ Implemented"]
        JSON["JSON Encoding<br/>(stdlib)<br/>✅ Implemented"]
        MsgPack["MessagePack<br/>(stub)<br/>🚧 Example"]
        Proto["Protobuf<br/>(future)<br/>💡 Future"]
    end
    
    subgraph Templates["Jinja2 Templates"]
        MsgTmpl["message.py.jinja2"]
        SetupTmpl["setup.py.jinja2"]
    end
    
    subgraph Output["Generated Code"]
        PyCDR["Python + CDR<br/>(ROS2 interop)"]
        PyJSON["Python + JSON<br/>(web APIs)"]
        PyMsgPack["Python + MessagePack<br/>(microservices)"]
        Future["Rust + CDR<br/>TypeScript + JSON<br/>etc."]
    end
    
    Discovery --> Parser
    Parser --> TypeHash
    TypeHash --> Lang
    
    Lang --> Templates
    Enc --> Templates
    
    Python -.uses.-> CDR
    Python -.uses.-> JSON
    Python -.uses.-> MsgPack
    
    Templates --> Output
    
    style CDR fill:#90EE90
    style JSON fill:#90EE90
    style MsgPack fill:#FFD700
    style Proto fill:#E0E0E0
    style Python fill:#4B9CD3
```

## Encoding Backend Architecture

```mermaid
classDiagram
    class EncodingBackend {
        <<abstract>>
        +get_name() str
        +get_python_dependencies() list
        +get_python_imports() list
        +get_python_serialize_method() str
        +get_python_deserialize_method() str
        +supports_type_hash() bool
        +supports_dds_type_name() bool
        +get_metadata() dict
    }
    
    class CDREncoding {
        +get_name() "cdr"
        +get_python_dependencies() ["pycdr2>=0.3.0"]
        +supports_type_hash() True
        +supports_dds_type_name() True
    }
    
    class JSONEncoding {
        +get_name() "json"
        +get_python_dependencies() []
        +supports_type_hash() False
        +supports_dds_type_name() False
    }
    
    class MessagePackEncoding {
        +get_name() "msgpack"
        +get_python_dependencies() ["msgpack>=1.0.0"]
        +supports_type_hash() False
        +supports_dds_type_name() False
    }
    
    class ProtobufEncoding {
        <<future>>
        +get_name() "protobuf"
        +get_python_dependencies() ["protobuf>=4.0.0"]
    }
    
    EncodingBackend <|-- CDREncoding
    EncodingBackend <|-- JSONEncoding
    EncodingBackend <|-- MessagePackEncoding
    EncodingBackend <|-- ProtobufEncoding
    
    class PythonGenerator {
        -encoding_backend EncodingBackend
        +generate(messages, output_dir)
        -_generate_message_file()
        -_get_python_type()
    }
    
    PythonGenerator --> EncodingBackend : uses
```

## Code Generation Flow

```mermaid
sequenceDiagram
    participant CLI as CLI/User
    participant Gen as Generator
    participant Lang as Language Backend<br/>(Python)
    participant Enc as Encoding Backend<br/>(CDR/JSON)
    participant Tmpl as Jinja2 Template
    participant Out as Generated File
    
    CLI->>Gen: generate(lang='python', encoding='cdr')
    Gen->>Gen: Discover .msg files
    Gen->>Gen: Parse message structures
    Gen->>Gen: Compute type hashes
    
    Gen->>Lang: PythonGenerator(encoding='cdr')
    Lang->>Enc: get_encoding('cdr')
    Enc-->>Lang: CDREncoding instance
    
    Gen->>Lang: generate(messages, output_dir)
    
    loop For each message
        Lang->>Enc: supports_type_hash()
        Enc-->>Lang: True
        Lang->>Enc: supports_dds_type_name()
        Enc-->>Lang: True
        
        Lang->>Tmpl: render(message, encoding_name='cdr',<br/>needs_type_hash=True,<br/>needs_dds_type_name=True)
        
        Tmpl->>Tmpl: {% if encoding_name == 'cdr' %}
        Tmpl->>Tmpl: Include pycdr2 imports
        Tmpl->>Tmpl: Add TYPE_HASH constant
        Tmpl->>Tmpl: Add DDS_TYPE_NAME constant
        Tmpl->>Tmpl: Generate serialize() using pycdr2
        
        Tmpl-->>Lang: Rendered Python code
        Lang->>Out: Write .py file
    end
    
    Out-->>CLI: Generated package ready
```

## Package Structure

```mermaid
graph TD
    Root["ros2_interface_generator/"]
    
    Root --> Core["ros2_interface_generator/"]
    Root --> Bin["bin/"]
    Root --> Tests["tests/"]
    Root --> Docs["Documentation"]
    
    Core --> Gen["generator.py<br/>(Core logic)"]
    Core --> PkgLists["package_lists.py<br/>(Presets)"]
    Core --> LangDir["languages/"]
    Core --> EncDir["encodings/"]
    Core --> TmplDir["templates/"]
    
    LangDir --> PyGen["python.py<br/>(Python generator)"]
    LangDir --> RustGen["rust.py<br/>(future)"]
    
    EncDir --> EncInit["__init__.py<br/>(Factory)"]
    EncDir --> EncBase["base.py<br/>(Abstract base)"]
    EncDir --> EncCDR["cdr.py<br/>(CDR impl ✅)"]
    EncDir --> EncJSON["json_encoding.py<br/>(JSON impl ✅)"]
    EncDir --> EncMsgPack["msgpack_encoding.py<br/>(Stub 🚧)"]
    
    TmplDir --> PyTmpl["python/"]
    PyTmpl --> MsgTmpl["message.py.jinja2"]
    PyTmpl --> SetupTmpl["setup.py.jinja2"]
    
    Bin --> CLI["ros2-generate-interfaces<br/>(CLI tool)"]
    
    Tests --> TestBasic["test_basic.py"]
    Tests --> TestPy["test_python_generation.py"]
    Tests --> TestE2E["test_end_to_end.py"]
    
    Docs --> README["README.md"]
    Docs --> EncArch["ENCODING_ARCHITECTURE.md"]
    Docs --> EncExample["ENCODING_EXAMPLE.md"]
    Docs --> FuturePf["FUTURE_PROOFING_SUMMARY.md"]
    
    style EncCDR fill:#90EE90
    style EncJSON fill:#90EE90
    style EncMsgPack fill:#FFD700
    style PyGen fill:#4B9CD3
```

## Message Generation Process

```mermaid
flowchart LR
    subgraph Input["Input"]
        MsgFiles[".msg files<br/>(geometry_msgs/Twist.msg)"]
    end
    
    subgraph Parse["Parsing"]
        Parser["Message Parser"]
        Fields["Extract Fields<br/>(linear, angular)"]
        Deps["Resolve Dependencies<br/>(Vector3)"]
    end
    
    subgraph Hash["Type Hash"]
        ROS2CLI["ros2 interface show<br/>--verbose"]
        Compute["Compute SHA-256<br/>(RIHS01)"]
    end
    
    subgraph Generate["Code Generation"]
        LangBackend["Language Backend<br/>(Python)"]
        EncBackend["Encoding Backend<br/>(CDR/JSON)"]
        Template["Jinja2 Template"]
    end
    
    subgraph Output["Output"]
        PyFile["twist.py<br/>(dataclass + serialize)"]
        TypeHashConst["TYPE_HASH = 'RIHS01_...'"]
        SerializeMethod["serialize() method"]
        DeserializeMethod["deserialize() method"]
    end
    
    MsgFiles --> Parser
    Parser --> Fields
    Fields --> Deps
    Deps --> ROS2CLI
    ROS2CLI --> Compute
    
    Compute --> LangBackend
    Fields --> LangBackend
    Deps --> LangBackend
    
    LangBackend --> EncBackend
    LangBackend --> Template
    EncBackend --> Template
    
    Template --> PyFile
    Template --> TypeHashConst
    Template --> SerializeMethod
    Template --> DeserializeMethod
```

## Encoding Selection Flow

```mermaid
flowchart TD
    Start([User runs<br/>ros2-generate-interfaces])
    
    Start --> CheckEnc{Encoding<br/>specified?}
    
    CheckEnc -->|Yes| GetEnc["get_encoding(name)"]
    CheckEnc -->|No| Default["Default: 'cdr'"]
    
    Default --> GetEnc
    
    GetEnc --> IsCDR{encoding == 'cdr'?}
    GetEnc --> IsJSON{encoding == 'json'?}
    GetEnc --> IsMsgPack{encoding == 'msgpack'?}
    
    IsCDR -->|Yes| CDRBackend["CDREncoding()<br/>- pycdr2<br/>- Type hashes ✅<br/>- DDS names ✅"]
    IsJSON -->|Yes| JSONBackend["JSONEncoding()<br/>- stdlib json<br/>- Type hashes ❌<br/>- DDS names ❌"]
    IsMsgPack -->|Yes| MsgPackBackend["MessagePackEncoding()<br/>- msgpack<br/>- Type hashes ❌<br/>- DDS names ❌"]
    
    CDRBackend --> Template
    JSONBackend --> Template
    MsgPackBackend --> Template
    
    Template{{"Jinja2 Template<br/>message_universal.py.jinja2"}}
    
    Template --> CondCDR{"{% if encoding_name == 'cdr' %}"}
    Template --> CondJSON{"{% elif encoding_name == 'json' %}"}
    
    CondCDR -->|True| GenCDR["from pycdr2 import IdlStruct<br/>class Msg(IdlStruct):<br/>  TYPE_HASH = ...<br/>  DDS_TYPE_NAME = ...<br/>  serialize() via pycdr2"]
    
    CondJSON -->|True| GenJSON["import json<br/>class Msg:<br/>  serialize() via json<br/>  (no type hash/DDS name)"]
    
    GenCDR --> Output([Generated<br/>Python file])
    GenJSON --> Output
    
    style CDRBackend fill:#90EE90
    style JSONBackend fill:#90EE90
    style MsgPackBackend fill:#FFD700
```

## Adding a New Encoding

```mermaid
flowchart TB
    Start([Want new encoding<br/>e.g., Protobuf])
    
    Start --> Step1["Step 1: Create Backend Class<br/>encodings/protobuf_encoding.py"]
    
    Step1 --> Impl["Implement EncodingBackend:<br/>• get_name() → 'protobuf'<br/>• get_python_dependencies() → ['protobuf>=4.0.0']<br/>• get_python_serialize_method()<br/>• get_python_deserialize_method()<br/>• supports_type_hash() → False<br/>• supports_dds_type_name() → False"]
    
    Impl --> Step2["Step 2: Register in __init__.py<br/>elif encoding_name == 'protobuf':<br/>  return ProtobufEncoding()"]
    
    Step2 --> Step3["Step 3: (Optional) Update Template<br/>Add {% elif encoding_name == 'protobuf' %}<br/>section if needed"]
    
    Step3 --> Done([Done! Use it:<br/>ros2-generate-interfaces<br/>-e protobuf])
    
    Done --> Example["Generated code automatically:<br/>• Uses protobuf serialization<br/>• No type hash (not for DDS)<br/>• No DDS type name<br/>• Custom serialize/deserialize"]
    
    style Step1 fill:#FFE4B5
    style Step2 fill:#FFE4B5
    style Step3 fill:#FFE4B5
    style Done fill:#90EE90
```

## Encoding Comparison Matrix

| Feature              | CDR ✅ | JSON ✅ | MessagePack 🚧 | Protobuf 💡 |
|----------------------|--------|---------|----------------|-------------|
| **Implementation**   | Full   | Full    | Stub/Example   | Future      |
| **Binary Format**    | ✅     | ❌      | ✅             | ✅          |
| **Compact**          | ✅     | ❌      | ✅             | ✅          |
| **Human Readable**   | ❌     | ✅      | ❌             | ❌          |
| **ROS2 Compatible**  | ✅     | ❌      | ❌             | ❌          |
| **Type Hash**        | ✅     | ❌      | ❌             | ❌          |
| **DDS Interop**      | ✅     | ❌      | ❌             | ❌          |
| **Zero Deps**        | ❌     | ✅      | ❌             | ❌          |
| **Schema-based**     | ✅     | ❌      | ❌             | ✅          |
| **Use Case**         | ROS2   | Web APIs| Microservices  | gRPC/Proto  |

## Key Design Principles

### 1. Separation of Concerns
- **Language backends** handle syntax and structure (Python dataclasses, Rust structs, etc.)
- **Encoding backends** handle serialization logic (CDR, JSON, MessagePack, etc.)
- Neither knows about the other's implementation details

### 2. Extensibility
- Adding a new language: Create class in `languages/`
- Adding a new encoding: Create class in `encodings/`
- Both follow abstract base classes

### 3. Flexibility
- Any language can use any encoding
- `Python + CDR` for ROS2 interop
- `Python + JSON` for web APIs
- `Rust + CDR` for performance
- `TypeScript + JSON` for browser apps

### 4. Template-driven
- Jinja2 templates receive encoding metadata
- Templates adapt to encoding capabilities
- `{% if needs_type_hash %}` conditionals

### 5. Type Safety
- Abstract base classes define contracts
- Type hints throughout
- Compile-time checking where possible

## Benefits

```mermaid
mindmap
    root((Modular<br/>Architecture))
        Maintainability
            Single responsibility
            Isolated changes
            Clear interfaces
        Extensibility
            New encodings: ~100 lines
            New languages: independent
            Mix and match
        Testing
            Test encodings separately
            Test languages separately
            Integration tests
        Performance
            Lazy loading
            Only generate what's needed
            Parallel generation possible
        Developer Experience
            Clear documentation
            Examples provided
            3-step process to extend
```

## Future Roadmap

1. **Complete MessagePack** - Finish the stub implementation
2. **Add Protobuf** - For gRPC and cross-platform
3. **Rust Backend** - Generate Rust code with CDR
4. **TypeScript Backend** - Generate TS code with JSON
5. **Service/Action Support** - Beyond just messages
6. **Performance Optimization** - Parallel generation, caching
7. **Plugin System** - User-defined backends

---

See also:
- [ENCODING_ARCHITECTURE.md](ENCODING_ARCHITECTURE.md) - Detailed architecture guide
- [ENCODING_EXAMPLE.md](ENCODING_EXAMPLE.md) - Quick-start for adding encodings
- [README.md](README.md) - User documentation

