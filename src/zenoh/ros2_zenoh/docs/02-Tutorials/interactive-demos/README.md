# Interactive Demos

**Live, executable demonstrations of ros2-zenoh concepts**

These interactive demos run directly in your markdown viewer (Obsidian with Dataview plugin) and visualize core ros2-zenoh behaviors.

---

## Available Demos

### 1. 🔄 [Rate Visualizer](rate-visualizer.md)

**Demonstrates**: Fixed-rate execution patterns

**Key Concepts**:
- Rate control algorithms
- Deadline detection
- Skip-if-behind behavior
- Efficiency metrics

**Interactive Features**:
- Adjust target rate (1-50 Hz)
- Vary processing time
- Watch real-time visualization
- See deadline misses

**Learn**: How `Rate` and `fixed_rate_loop()` maintain consistent timing

---

### 2. ⏰ [Clock Modes](clock-modes.md)

**Demonstrates**: Three time management modes

**Key Concepts**:
- WALL_TIME (real system time)
- SIM_TIME_LIVE (from /clock topic)
- SIM_TIME_TEST (manual control)

**Interactive Features**:
- Start/stop all clocks
- Manually advance test time
- Timeline visualization
- Mode comparison

**Learn**: When to use each time mode and how they behave differently

---

### 3. 📬 [Subscription Depth](subscription-depth.md)

**Demonstrates**: Ring buffer message queue behavior

**Key Concepts**:
- Queue depth (maxlen)
- Ring buffer semantics
- Message dropping (oldest first)
- Backpressure handling

**Interactive Features**:
- Adjust queue depth
- Control publish/process rates
- Watch messages drop
- Monitor statistics

**Learn**: How subscription depth prevents memory issues and keeps data fresh

---

## Requirements

### For Obsidian Users

**Required Plugin**:
- [Dataview](https://github.com/blacksmithgu/obsidian-dataview)

**Installation**:
1. Settings → Community Plugins → Browse
2. Search "Dataview"
3. Install + Enable

### For Other Markdown Viewers

These demos use JavaScript and HTML5 Canvas, which require:
- JavaScript execution support
- Canvas API support
- DOM manipulation

**Tested with**:
- ✅ Obsidian (with Dataview plugin)
- ⚠️ VS Code (limited - static view only)
- ❌ GitHub (no JS execution)

---

## How to Use

1. **Open in Obsidian** with Dataview plugin enabled
2. **Switch to Reading View** (preview mode)
3. **Interact with controls** - sliders, buttons work live
4. **Experiment** with different parameters
5. **Observe** visualizations update in real-time

---

## What's Happening Under the Hood

### Technologies

- **JavaScript**: Dataview plugin's `dataviewjs` blocks
- **Canvas API**: For timeline and chart visualization
- **HTML5**: DOM manipulation for UI controls
- **CSS**: Styling and animations

### Limitations

- **Not actual ros2-zenoh code** - Simplified simulations
- **Doesn't use Zenoh transport** - Local mock only
- **Educational purpose** - Demonstrates concepts, not production code

---

## From Demo to Code

Each demo shows equivalent Python code that produces the same behavior with the actual ros2-zenoh library.

### Example: Rate Demo → Production Code

**What you see in demo**:
```javascript
// Simulated rate control
const period = 1.0 / targetHz;
if (elapsedTime > period) {
    // Deadline missed!
}
```

**What you write in production**:
```python
from ros2_zenoh_python import fixed_rate_loop

async for tick in fixed_rate_loop(10.0, clock):
    await process_data()
    
    if tick.missed_deadline:
        print(f"Overrun: {tick.overrun * 1000:.1f}ms")
```

**Same behavior, production-ready!**

---

## Contributing New Demos

Want to add more interactive demos? Follow this template:

```markdown
# Demo Title

**Demonstrates**: What concept

```dataviewjs
const app = dv.container;

app.innerHTML = `
    <!-- Your HTML UI here -->
`;

// Your JavaScript logic here
```

**Key concept explanation**

**Code equivalent in Python**
```

---

## Feedback

Found a bug in a demo? Have an idea for a new one?

Create an issue or discussion in the project repository!

---

## Future Demos (Planned)

- [ ] Publisher/Subscriber Message Flow
- [ ] Service Call Round-Trip
- [ ] Action State Machine
- [ ] QoS Profile Effects
- [ ] Serialization Comparison (CDR vs JSON vs MessagePack)
- [ ] Type Hash Computation
- [ ] Namespace Isolation Demo

---

## Learn More

- [[../README|Tutorials Index]] - All tutorials
- [[../../06-Specs/README|Specifications]] - Technical specs
- [[../../05-Patterns/README|Patterns]] - Code patterns

---

**✨ These demos make learning interactive and fun!**

Open any demo file in Obsidian Reading View to start exploring.


