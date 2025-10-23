# Interactive Subscription Depth Demo

**Visualize ring buffer behavior and message queue management**

This demo shows how subscription depth affects message handling when messages arrive faster than they can be processed.

---

## Try It: Ring Buffer Simulation

```dataviewjs
const app = dv.container;

app.innerHTML = `
<div style="padding: 1.5em; border: 2px solid #FF5722; border-radius: 8px; background: #f8f9fa;">
    <h3 style="margin-top: 0; color: #D84315;">📬 Subscription Depth & Ring Buffer</h3>
    
    <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 1em; margin: 1.5em 0;">
        <div>
            <label style="display: block; margin-bottom: 0.5em;">
                <strong>Queue Depth (maxlen):</strong>
                <input type="range" id="depth" min="1" max="20" value="5" style="width: 100%;">
                <span id="depth-val" style="color: #FF5722; font-weight: bold;">5</span> messages
            </label>
        </div>
        
        <div>
            <label style="display: block; margin-bottom: 0.5em;">
                <strong>Message Rate:</strong>
                <input type="range" id="pub-rate" min="1" max="20" value="10" style="width: 100%;">
                <span id="rate-val" style="color: #FF9800; font-weight: bold;">10</span> msg/s
            </label>
        </div>
    </div>
    
    <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 1em; margin: 1em 0;">
        <div>
            <label style="display: block; margin-bottom: 0.5em;">
                <strong>Processing Speed:</strong>
                <input type="range" id="proc-speed" min="1" max="20" value="8" style="width: 100%;">
                <span id="speed-val" style="color: #4CAF50; font-weight: bold;">8</span> msg/s
            </label>
        </div>
        
        <div style="padding: 1em; background: white; border-radius: 4px; text-align: center;">
            <div style="font-size: 12px; color: #666;">Buffer Status</div>
            <div id="buffer-status" style="font-size: 20px; font-weight: bold; color: #4CAF50;">Ready</div>
        </div>
    </div>
    
    <div style="text-align: center; margin: 1em 0;">
        <button id="run-btn" style="padding: 10px 24px; font-size: 16px; background: #4CAF50; color: white; border: none; border-radius: 4px; cursor: pointer; margin-right: 10px;">
            ▶️ Start Simulation
        </button>
        <button id="stop-btn" style="padding: 10px 24px; font-size: 16px; background: #F44336; color: white; border: none; border-radius: 4px; cursor: pointer; margin-right: 10px;" disabled>
            ⏹️ Stop
        </button>
        <button id="reset-btn" style="padding: 10px 24px; font-size: 16px; background: #9E9E9E; color: white; border: none; border-radius: 4px; cursor: pointer;">
            🔄 Reset
        </button>
    </div>
    
    <div style="background: white; border: 1px solid #ddd; border-radius: 8px; padding: 1.5em; margin: 1em 0;">
        <h4 style="margin: 0 0 1em 0;">Queue Visualization</h4>
        <div id="queue-viz" style="display: flex; gap: 5px; min-height: 80px; align-items: flex-end; padding: 10px; background: #f5f5f5; border-radius: 4px;">
            <div style="color: #999; margin: auto;">Queue empty - click Start</div>
        </div>
    </div>
    
    <div style="display: grid; grid-template-columns: 2fr 1fr; gap: 1em; margin-top: 1em;">
        <div id="stats" style="font-family: 'Courier New', monospace; font-size: 13px; background: #263238; color: #4CAF50; padding: 1em; border-radius: 4px; min-height: 150px;"></div>
        
        <div style="padding: 1em; background: #FAFAFA; border-radius: 4px;">
            <h5 style="margin: 0 0 0.5em 0;">Legend</h5>
            <div style="margin-bottom: 0.5em; font-size: 12px;">
                <span style="display: inline-block; width: 20px; height: 20px; background: #2196F3; vertical-align: middle;"></span>
                <span style="margin-left: 8px;">Queued</span>
            </div>
            <div style="margin-bottom: 0.5em; font-size: 12px;">
                <span style="display: inline-block; width: 20px; height: 20px; background: #4CAF50; vertical-align: middle;"></span>
                <span style="margin-left: 8px;">Processing</span>
            </div>
            <div style="margin-bottom: 0.5em; font-size: 12px;">
                <span style="display: inline-block; width: 20px; height: 20px; background: #F44336; vertical-align: middle;"></span>
                <span style="margin-left: 8px;">Dropped</span>
            </div>
            <div style="margin-top: 1em; padding-top: 1em; border-top: 1px solid #ddd; font-size: 11px; color: #666;">
                <strong>Ring Buffer:</strong><br>
                Oldest messages dropped when full
            </div>
        </div>
    </div>
</div>
`;

const depthSlider = app.querySelector('#depth');
const pubRateSlider = app.querySelector('#pub-rate');
const procSpeedSlider = app.querySelector('#proc-speed');
const depthVal = app.querySelector('#depth-val');
const rateVal = app.querySelector('#rate-val');
const speedVal = app.querySelector('#speed-val');
const runBtn = app.querySelector('#run-btn');
const stopBtn = app.querySelector('#stop-btn');
const resetBtn = app.querySelector('#reset-btn');
const stats = app.querySelector('#stats');
const queueViz = app.querySelector('#queue-viz');
const bufferStatus = app.querySelector('#buffer-status');

let isRunning = false;
let shouldStop = false;
let messageQueue = [];
let maxDepth = 5;
let nextMsgId = 1;
let statsData = {
    published: 0,
    processed: 0,
    dropped: 0
};

// Update labels
depthSlider.addEventListener('input', (e) => {
    depthVal.textContent = e.target.value;
});

pubRateSlider.addEventListener('input', (e) => {
    rateVal.textContent = e.target.value;
});

procSpeedSlider.addEventListener('input', (e) => {
    speedVal.textContent = e.target.value;
});

// Visualize queue
function updateQueueViz() {
    queueViz.innerHTML = '';
    
    if (messageQueue.length === 0) {
        queueViz.innerHTML = '<div style="color: #999; margin: auto;">Queue empty</div>';
        bufferStatus.textContent = 'Empty';
        bufferStatus.style.color = '#999';
        return;
    }
    
    // Show queue depth limit indicators
    for (let i = 0; i < maxDepth; i++) {
        const slot = document.createElement('div');
        slot.style.cssText = `
            width: ${Math.max(30, 500 / maxDepth)}px;
            height: 60px;
            border: 2px dashed #ddd;
            border-radius: 4px;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 10px;
            color: #999;
        `;
        slot.textContent = `${i + 1}`;
        queueViz.appendChild(slot);
    }
    
    // Overlay actual messages
    queueViz.innerHTML = '';
    messageQueue.forEach((msg, idx) => {
        const msgEl = document.createElement('div');
        msgEl.style.cssText = `
            width: ${Math.max(30, 500 / maxDepth)}px;
            height: 60px;
            background: ${msg.processing ? '#4CAF50' : '#2196F3'};
            color: white;
            border-radius: 4px;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 12px;
            font-weight: bold;
            box-shadow: 0 2px 4px rgba(0,0,0,0.2);
            animation: slideIn 0.3s ease-out;
        `;
        msgEl.innerHTML = `
            <div>
                <div style="font-size: 16px;">#${msg.id}</div>
                <div style="font-size: 10px; opacity: 0.8;">${msg.processing ? 'Processing' : 'Queued'}</div>
            </div>
        `;
        queueViz.appendChild(msgEl);
    });
    
    // Buffer status
    const utilization = (messageQueue.length / maxDepth * 100).toFixed(0);
    bufferStatus.textContent = `${messageQueue.length}/${maxDepth} (${utilization}%)`;
    
    if (messageQueue.length >= maxDepth) {
        bufferStatus.style.color = '#F44336';
    } else if (messageQueue.length >= maxDepth * 0.7) {
        bufferStatus.style.color = '#FF9800';
    } else {
        bufferStatus.style.color = '#4CAF50';
    }
}

// Update stats display
function updateStats() {
    const dropRate = statsData.published > 0 
        ? (statsData.dropped / statsData.published * 100).toFixed(1)
        : 0;
    
    stats.innerHTML = `
> Messages Published: ${statsData.published}
> Messages Processed: ${statsData.processed}
> Messages Dropped: ${statsData.dropped}
> 
> Drop Rate: ${dropRate}%
> Queue Length: ${messageQueue.length}/${maxDepth}
> Next Message ID: #${nextMsgId}
    `.trim();
}

updateStats();
updateQueueViz();

// Simulation
async function runSimulation() {
    if (isRunning) return;
    
    isRunning = true;
    shouldStop = false;
    runBtn.disabled = true;
    stopBtn.disabled = false;
    depthSlider.disabled = true;
    pubRateSlider.disabled = false;
    procSpeedSlider.disabled = false;
    
    maxDepth = Number(depthSlider.value);
    const pubRate = Number(pubRateSlider.value);
    const procSpeed = Number(procSpeedSlider.value);
    
    const pubInterval = 1000 / pubRate;  // ms between publishes
    const procInterval = 1000 / procSpeed;  // ms between processing
    
    // Publisher loop
    const publisherLoop = setInterval(() => {
        if (shouldStop) {
            clearInterval(publisherLoop);
            return;
        }
        
        // Create new message
        const msg = {
            id: nextMsgId++,
            processing: false
        };
        
        // Try to add to queue
        if (messageQueue.length >= maxDepth) {
            // Ring buffer: drop oldest
            const dropped = messageQueue.shift();
            statsData.dropped++;
            
            // Show dropped message briefly
            const droppedEl = document.createElement('div');
            droppedEl.style.cssText = `
                position: absolute;
                top: -30px;
                left: 0;
                background: #F44336;
                color: white;
                padding: 4px 8px;
                border-radius: 4px;
                font-size: 12px;
                animation: fadeOut 1s ease-out;
            `;
            droppedEl.textContent = `#${dropped.id} dropped`;
            queueViz.style.position = 'relative';
            queueViz.appendChild(droppedEl);
            setTimeout(() => droppedEl.remove(), 1000);
        }
        
        messageQueue.push(msg);
        statsData.published++;
        
        updateQueueViz();
        updateStats();
    }, pubInterval);
    
    // Consumer loop
    const consumerLoop = setInterval(() => {
        if (shouldStop) {
            clearInterval(consumerLoop);
            return;
        }
        
        if (messageQueue.length > 0) {
            // Mark as processing
            messageQueue[0].processing = true;
            updateQueueViz();
            
            // Process (takes time)
            setTimeout(() => {
                if (messageQueue.length > 0) {
                    messageQueue.shift();
                    statsData.processed++;
                    updateQueueViz();
                    updateStats();
                }
            }, procInterval * 0.8);
        }
    }, procInterval);
    
    // Stop handler
    stopBtn.addEventListener('click', () => {
        shouldStop = true;
        clearInterval(publisherLoop);
        clearInterval(consumerLoop);
        
        isRunning = false;
        runBtn.disabled = false;
        stopBtn.disabled = true;
        depthSlider.disabled = false;
    }, { once: true });
}

resetBtn.addEventListener('click', () => {
    shouldStop = true;
    isRunning = false;
    messageQueue = [];
    nextMsgId = 1;
    statsData = { published: 0, processed: 0, dropped: 0 };
    
    runBtn.disabled = false;
    stopBtn.disabled = true;
    depthSlider.disabled = false;
    pubRateSlider.disabled = false;
    procSpeedSlider.disabled = false;
    
    updateQueueViz();
    updateStats();
});

runBtn.addEventListener('click', runSimulation);

// Add CSS animation
const style = document.createElement('style');
style.textContent = `
@keyframes slideIn {
    from { transform: translateY(-20px); opacity: 0; }
    to { transform: translateY(0); opacity: 1; }
}
@keyframes fadeOut {
    to { opacity: 0; transform: translateY(-20px); }
}
`;
document.head.appendChild(style);
```

---

## Understanding Ring Buffer Behavior

### What Happens When Queue is Full?

```python
from collections import deque

# Ring buffer with maxlen
buffer = deque(maxlen=5)

# Add messages
buffer.append("msg1")  # [msg1]
buffer.append("msg2")  # [msg1, msg2]
# ... fill to capacity ...
buffer.append("msg5")  # [msg1, msg2, msg3, msg4, msg5]  FULL!

# Next message drops oldest
buffer.append("msg6")  # [msg2, msg3, msg4, msg5, msg6]  msg1 dropped!
```

---

## Scenarios to Try

### 1. **Balanced System** ✅
- Depth: 5
- Publish Rate: 8 msg/s
- Processing Speed: 10 msg/s
- **Result**: Queue stays small, no drops

### 2. **Slow Consumer** ⚠️
- Depth: 5
- Publish Rate: 15 msg/s
- Processing Speed: 8 msg/s
- **Result**: Queue fills up, messages dropped

### 3. **Bursty Traffic** 📊
- Depth: 10 (larger buffer)
- Publish Rate: 20 msg/s
- Processing Speed: 15 msg/s
- **Result**: Buffer absorbs bursts, some drops

### 4. **Tiny Buffer** 🚨
- Depth: 1
- Publish Rate: 10 msg/s
- Processing Speed: 5 msg/s
- **Result**: Aggressive dropping, only latest message kept

---

## Code Equivalent

```python
from collections import deque
from ros2_zenoh_python import Subscription

class MyNode:
    def __init__(self):
        # Create subscription with depth
        self.sub = Subscription(
            Image, 
            "/camera",
            self.on_image,
            depth=10  # Ring buffer with 10 slots
        )
    
    async def on_image(self, msg):
        # If processing is slow and messages arrive fast,
        # older messages are automatically dropped
        await slow_processing(msg)
```

### Behavior

**If messages arrive at 30 Hz but you process at 10 Hz:**
- Queue fills to depth (10 messages)
- New messages drop oldest messages
- **You always process recent data** (not stale!)

---

## Design Considerations

### When to Use Small Depth (1-5)

✅ **Sensor data** (camera, lidar)
- Always want latest frame
- Old data is stale

✅ **Real-time control**
- Latest state matters most
- Historical data less important

### When to Use Large Depth (10-100)

✅ **Commands**
- Don't want to drop user commands
- Process all inputs

✅ **Logging/Recording**
- Want complete history
- Can tolerate delay

### When to Use Unbounded (None)

⚠️ **Use carefully!**
- No automatic dropping
- Can grow without limit
- Risk of memory exhaustion

**Better**: Use large depth + monitoring

---

## Best Practices

```python
# ✅ Good: Appropriate depth for use case
camera_sub = Subscription(
    Image, 
    "/camera",
    self.on_image,
    depth=1  # Only latest frame needed
)

command_sub = Subscription(
    Twist,
    "/cmd_vel", 
    self.on_command,
    depth=10  # Don't drop commands
)

# ⚠️ Risky: No depth limit
log_sub = Subscription(
    Log,
    "/rosout",
    self.on_log,
    depth=None  # Can grow forever!
)

# ✅ Better: Large but bounded
log_sub = Subscription(
    Log,
    "/rosout",
    self.on_log,
    depth=1000  # Bounded buffer
)
```

---

## Learn More

- [[subscription|Subscription Specification]] - Full subscription behavior
- [[ADR-003]] - Ring buffer decision rationale
- [[publisher|Publisher Specification]] - Message publishing

---

**Key Takeaway**: Ring buffers ensure you always process recent data, automatically dropping stale messages when processing can't keep up.


