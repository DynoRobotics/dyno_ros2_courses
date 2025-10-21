# Template Simplification

## What Changed

Previously we had two templates:
- `message.py.jinja2` - CDR-specific template (legacy)
- `message_universal.py.jinja2` - Multi-encoding template

This was redundant since the multi-encoding template already handled CDR perfectly.

## After Simplification

Now we have just **one template**:
- `message.py.jinja2` - Handles **all** encodings (CDR, JSON, MessagePack, etc.)

No need for "universal" prefix - if there's only one, it's the standard.

## Why This is Better

### ❌ Before (Redundant)
```python
# In python.py:
try:
    template = self.env.get_template("message_universal.py.jinja2")
except:
    template = self.env.get_template("message.py.jinja2")  # Fallback
```

Two templates doing essentially the same thing for CDR.

### ✅ After (Clean)
```python
# In python.py:
template = self.env.get_template("message.py.jinja2")
```

One template with a standard name, clean and simple.

## Template Logic

The universal template uses simple conditionals:

```jinja2
{% if encoding_name == 'cdr' %}
from pycdr2 import IdlStruct
class Message(IdlStruct, typename="..."):
    TYPE_HASH = "..."
    DDS_TYPE_NAME = "..."
    def serialize(self): return IdlStruct.serialize(self)

{% elif encoding_name == 'json' %}
import json
class Message:
    def serialize(self): return json.dumps(self.to_dict()).encode('utf-8')

{% elif encoding_name == 'msgpack' %}
import msgpack
class Message:
    def serialize(self): return msgpack.packb(self.to_dict())
{% endif %}
```

## Benefits

1. **Single Source of Truth**: One template for all encodings
2. **No Special Cases**: CDR is just another encoding, not a "default with fallback"
3. **Easier Maintenance**: Update one file, not two
4. **Clearer Intent**: The template name says "universal" - it means it
5. **Less Confusion**: No wondering which template is used when

## Design Principle

**No encoding is "special"** - they're all equal citizens handled by the same template system. CDR happens to be commonly used for ROS2, but architecturally it's just one encoding option among many.

This follows the **Open/Closed Principle**:
- ✅ Open for extension (add new encodings easily)
- ✅ Closed for modification (don't need to change core template logic)

