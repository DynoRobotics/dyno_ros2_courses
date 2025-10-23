# Interactive Clock Modes Demo

**Visualize the three time modes: WALL_TIME, SIM_TIME_LIVE, and SIM_TIME_TEST**

This demo shows how different time modes behave and when to use each.

---

## Try It: Clock Mode Comparison

```dataviewjs
const app = dv.container;

app.innerHTML = `
<div style="padding: 1.5em; border: 2px solid #9C27B0; border-radius: 8px; background: #f8f9fa;">
    <h3 style="margin-top: 0; color: #7B1FA2;">⏰ Clock Time Modes</h3>
    
    <div style="display: grid; grid-template-columns: repeat(3, 1fr); gap: 1em; margin: 1.5em 0;">
        <!-- WALL_TIME -->
        <div style="border: 2px solid #4CAF50; border-radius: 8px; padding: 1em; background: white;">
            <h4 style="margin: 0 0 0.5em 0; color: #4CAF50;">🌍 WALL_TIME</h4>
            <div id="wall-time" style="font-family: 'Courier New', monospace; font-size: 24px; font-weight: bold; color: #2E7D32; text-align: center; margin: 1em 0;">
                0.000s
            </div>
            <p style="font-size: 12px; color: #666; margin: 0;">
                Real system time<br>
                <strong>Use for:</strong> Production
            </p>
        </div>
        
        <!-- SIM_TIME_LIVE -->
        <div style="border: 2px solid #FF9800; border-radius: 8px; padding: 1em; background: white;">
            <h4 style="margin: 0 0 0.5em 0; color: #FF9800;">📡 SIM_TIME_LIVE</h4>
            <div id="sim-live-time" style="font-family: 'Courier New', monospace; font-size: 24px; font-weight: bold; color: #E65100; text-align: center; margin: 1em 0;">
                0.000s
            </div>
            <p style="font-size: 12px; color: #666; margin: 0;">
                From /clock topic<br>
                <strong>Use for:</strong> Gazebo, Sim
            </p>
        </div>
        
        <!-- SIM_TIME_TEST -->
        <div style="border: 2px solid #2196F3; border-radius: 8px; padding: 1em; background: white;">
            <h4 style="margin: 0 0 0.5em 0; color: #2196F3;">🧪 SIM_TIME_TEST</h4>
            <div id="sim-test-time" style="font-family: 'Courier New', monospace; font-size: 24px; font-weight: bold; color: #1565C0; text-align: center; margin: 1em 0;">
                0.000s
            </div>
            <p style="font-size: 12px; color: #666; margin: 0;">
                Manual control<br>
                <strong>Use for:</strong> Unit tests
            </p>
        </div>
    </div>
    
    <div style="text-align: center; margin: 1.5em 0;">
        <button id="start-btn" style="padding: 10px 24px; font-size: 16px; background: #4CAF50; color: white; border: none; border-radius: 4px; cursor: pointer; margin-right: 10px;">
            ▶️ Start All
        </button>
        <button id="stop-btn" style="padding: 10px 24px; font-size: 16px; background: #F44336; color: white; border: none; border-radius: 4px; cursor: pointer; margin-right: 10px;" disabled>
            ⏹️ Stop
        </button>
        <button id="reset-btn" style="padding: 10px 24px; font-size: 16px; background: #9E9E9E; color: white; border: none; border-radius: 4px; cursor: pointer;">
            🔄 Reset
        </button>
    </div>
    
    <div style="background: white; border: 1px solid #ddd; border-radius: 8px; padding: 1em; margin: 1em 0;">
        <h4 style="margin: 0 0 1em 0;">🧪 SIM_TIME_TEST Controls (Manual)</h4>
        <div style="display: flex; gap: 10px; align-items: center;">
            <button id="advance-small" style="padding: 8px 16px; background: #2196F3; color: white; border: none; border-radius: 4px; cursor: pointer;">
                +0.1s
            </button>
            <button id="advance-medium" style="padding: 8px 16px; background: #2196F3; color: white; border: none; border-radius: 4px; cursor: pointer;">
                +1.0s
            </button>
            <button id="advance-large" style="padding: 8px 16px; background: #2196F3; color: white; border: none; border-radius: 4px; cursor: pointer;">
                +5.0s
            </button>
            <label style="margin-left: auto;">
                Custom: <input type="number" id="custom-advance" value="0.5" step="0.1" min="0" style="width: 80px; padding: 4px;">s
                <button id="advance-custom" style="padding: 8px 16px; background: #2196F3; color: white; border: none; border-radius: 4px; cursor: pointer; margin-left: 5px;">
                    Advance
                </button>
            </label>
        </div>
    </div>
    
    <canvas id="timeline" width="700" height="200" style="border: 1px solid #ddd; margin: 1em 0; background: white; display: block; width: 100%;"></canvas>
    
    <div style="background: #FAFAFA; border-radius: 8px; padding: 1em; margin-top: 1em;">
        <h4 style="margin: 0 0 0.5em 0;">📊 Key Differences</h4>
        <table style="width: 100%; font-size: 13px;">
            <tr style="background: #E8F5E9;">
                <td style="padding: 8px;"><strong>WALL_TIME</strong></td>
                <td style="padding: 8px;">Advances automatically with real time</td>
                <td style="padding: 8px;">✅ Always available</td>
            </tr>
            <tr style="background: #FFF3E0;">
                <td style="padding: 8px;"><strong>SIM_TIME_LIVE</strong></td>
                <td style="padding: 8px;">Advances with /clock messages</td>
                <td style="padding: 8px;">⚠️ Requires clock publisher</td>
            </tr>
            <tr style="background: #E3F2FD;">
                <td style="padding: 8px;"><strong>SIM_TIME_TEST</strong></td>
                <td style="padding: 8px;">Advances only when you call advance_by()</td>
                <td style="padding: 8px;">🧪 Perfect for tests</td>
            </tr>
        </table>
    </div>
</div>
`;

const wallTimeEl = app.querySelector('#wall-time');
const simLiveTimeEl = app.querySelector('#sim-live-time');
const simTestTimeEl = app.querySelector('#sim-test-time');
const startBtn = app.querySelector('#start-btn');
const stopBtn = app.querySelector('#stop-btn');
const resetBtn = app.querySelector('#reset-btn');
const canvas = app.querySelector('#timeline');
const ctx = canvas.getContext('2d');

// Time state
let wallTimeStart = null;
let simLiveTime = 0;
let simTestTime = 0;
let isRunning = false;
let animationFrame = null;

// History for timeline
const history = {
    wall: [],
    simLive: [],
    simTest: []
};

// Draw timeline
function drawTimeline() {
    ctx.clearRect(0, 0, 700, 200);
    
    // Background
    ctx.fillStyle = '#f5f5f5';
    ctx.fillRect(0, 0, 700, 200);
    
    // Grid
    ctx.strokeStyle = '#e0e0e0';
    ctx.lineWidth = 1;
    for (let i = 0; i <= 10; i++) {
        const x = 50 + i * 60;
        ctx.beginPath();
        ctx.moveTo(x, 20);
        ctx.lineTo(x, 180);
        ctx.stroke();
    }
    
    // Axes
    ctx.strokeStyle = '#333';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(50, 180);
    ctx.lineTo(650, 180);
    ctx.stroke();
    
    // Labels
    ctx.fillStyle = '#333';
    ctx.font = '12px sans-serif';
    ctx.textAlign = 'center';
    for (let i = 0; i <= 10; i++) {
        const x = 50 + i * 60;
        ctx.fillText(`${i}s`, x, 195);
    }
    
    // Draw time lines
    function drawLine(data, color, y) {
        if (data.length < 2) return;
        
        ctx.strokeStyle = color;
        ctx.lineWidth = 3;
        ctx.beginPath();
        
        data.forEach((point, i) => {
            const x = 50 + (point.realTime / 10) * 600;
            const lineY = y;
            
            if (i === 0) {
                ctx.moveTo(x, lineY);
            } else {
                ctx.lineTo(x, lineY);
            }
        });
        
        ctx.stroke();
        
        // Current position
        if (data.length > 0) {
            const last = data[data.length - 1];
            const x = 50 + (last.realTime / 10) * 600;
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.arc(x, y, 5, 0, Math.PI * 2);
            ctx.fill();
        }
    }
    
    drawLine(history.wall, '#4CAF50', 50);
    drawLine(history.simLive, '#FF9800', 90);
    drawLine(history.simTest, '#2196F3', 130);
    
    // Legend
    ctx.font = 'bold 12px sans-serif';
    ctx.textAlign = 'left';
    ctx.fillStyle = '#4CAF50';
    ctx.fillText('● WALL_TIME', 60, 45);
    ctx.fillStyle = '#FF9800';
    ctx.fillText('● SIM_TIME_LIVE', 60, 85);
    ctx.fillStyle = '#2196F3';
    ctx.fillText('● SIM_TIME_TEST', 60, 125);
}

drawTimeline();

// Update displays
function update() {
    if (!isRunning && animationFrame) return;
    
    // WALL_TIME - always advancing
    if (wallTimeStart) {
        const wallTime = (Date.now() - wallTimeStart) / 1000;
        wallTimeEl.textContent = wallTime.toFixed(3) + 's';
        
        if (wallTime <= 10) {
            history.wall.push({ realTime: wallTime, simTime: wallTime });
        }
    }
    
    // SIM_TIME_LIVE - simulates /clock topic (advances slower)
    if (isRunning) {
        simLiveTime += 0.016 * 0.5;  // 50% real-time
        simLiveTimeEl.textContent = simLiveTime.toFixed(3) + 's';
        
        const realTime = (Date.now() - wallTimeStart) / 1000;
        if (realTime <= 10) {
            history.simLive.push({ realTime, simTime: simLiveTime });
        }
    }
    
    // SIM_TIME_TEST - only advances manually
    simTestTimeEl.textContent = simTestTime.toFixed(3) + 's';
    
    drawTimeline();
    
    if (isRunning) {
        animationFrame = requestAnimationFrame(update);
    }
}

// Controls
startBtn.addEventListener('click', () => {
    if (isRunning) return;
    
    isRunning = true;
    if (!wallTimeStart) {
        wallTimeStart = Date.now();
    } else {
        // Resume
        wallTimeStart = Date.now() - (history.wall[history.wall.length - 1]?.simTime * 1000 || 0);
    }
    
    startBtn.disabled = true;
    stopBtn.disabled = false;
    
    update();
});

stopBtn.addEventListener('click', () => {
    isRunning = false;
    startBtn.disabled = false;
    stopBtn.disabled = true;
    
    if (animationFrame) {
        cancelAnimationFrame(animationFrame);
        animationFrame = null;
    }
});

resetBtn.addEventListener('click', () => {
    isRunning = false;
    wallTimeStart = null;
    simLiveTime = 0;
    simTestTime = 0;
    
    history.wall = [];
    history.simLive = [];
    history.simTest = [];
    
    wallTimeEl.textContent = '0.000s';
    simLiveTimeEl.textContent = '0.000s';
    simTestTimeEl.textContent = '0.000s';
    
    startBtn.disabled = false;
    stopBtn.disabled = true;
    
    if (animationFrame) {
        cancelAnimationFrame(animationFrame);
        animationFrame = null;
    }
    
    drawTimeline();
});

// SIM_TIME_TEST manual controls
function advanceTestTime(amount) {
    simTestTime += amount;
    simTestTimeEl.textContent = simTestTime.toFixed(3) + 's';
    
    const realTime = wallTimeStart ? (Date.now() - wallTimeStart) / 1000 : simTestTime;
    if (realTime <= 10) {
        history.simTest.push({ realTime, simTime: simTestTime });
    }
    
    drawTimeline();
}

app.querySelector('#advance-small').addEventListener('click', () => advanceTestTime(0.1));
app.querySelector('#advance-medium').addEventListener('click', () => advanceTestTime(1.0));
app.querySelector('#advance-large').addEventListener('click', () => advanceTestTime(5.0));
app.querySelector('#advance-custom').addEventListener('click', () => {
    const amount = parseFloat(app.querySelector('#custom-advance').value);
    if (!isNaN(amount) && amount > 0) {
        advanceTestTime(amount);
    }
});
```

---

## Understanding Time Modes

### 🌍 WALL_TIME
**Real system time - always advancing**

```python
clock = Clock(TimeMode.WALL_TIME)
print(clock.now())  # e.g., 1698345678.123
await asyncio.sleep(1.0)
print(clock.now())  # 1 second later
```

**When to use:**
- Production code
- Real-time applications
- When you need actual wall clock time

### 📡 SIM_TIME_LIVE
**Driven by `/clock` topic**

```python
# Publishes simulation time
clock_pub = Publisher(Clock, "/clock")

# Subscribes to simulation time
clock = Clock(TimeMode.SIM_TIME_LIVE, clock_sub)
print(clock.now())  # Time from last /clock message
```

**When to use:**
- Gazebo simulations
- Isaac Sim
- Any simulator that publishes `/clock`
- Testing with recorded data (bag files)

**Key feature:** Can run faster or slower than real-time!

### 🧪 SIM_TIME_TEST
**Manual control - only advances when you say so**

```python
clock = Clock(TimeMode.SIM_TIME_TEST, initial_time=0.0)
print(clock.now())  # 0.0

# Time doesn't advance automatically!
await asyncio.sleep(1.0)
print(clock.now())  # Still 0.0!

# You control time
clock.advance_by(5.0)
print(clock.now())  # 5.0
```

**When to use:**
- Unit tests (runs instantly!)
- Deterministic testing
- Time travel scenarios
- Fast-forwarding through long operations

---

## Key Observations

### Watch the Timeline Graph

1. **Green line (WALL_TIME)**: Advances steadily at real-time rate
2. **Orange line (SIM_TIME_LIVE)**: Advances at 50% speed (simulated)
3. **Blue line (SIM_TIME_TEST)**: Only advances when you click buttons

### Try This

1. Click "Start All"
   - WALL_TIME advances automatically
   - SIM_TIME_LIVE advances slowly
   - SIM_TIME_TEST stays frozen

2. Click "+1.0s" for SIM_TIME_TEST
   - Only test time jumps forward
   - Others keep their pace

3. Click "Stop"
   - WALL_TIME graph stops
   - You can still advance TEST time manually

---

## Test Code Example

```python
import pytest
from ros2_zenoh_python import Clock, TimeMode, Rate

@pytest.mark.asyncio
async def test_rate_with_sim_time():
    """Tests run INSTANTLY with SIM_TIME_TEST!"""
    clock = Clock(TimeMode.SIM_TIME_TEST, initial_time=0.0)
    rate = Rate(10.0, clock)  # 10 Hz
    
    # Simulate 10 seconds of operation
    for i in range(100):
        # Process data here
        await rate.sleep()
    
    # Verify exact timing (deterministic!)
    assert clock.now() == 10.0  # Exactly 10 seconds
    # This test runs in milliseconds, not 10 seconds!
```

---

## Learn More

- [[clock|Clock Specification]] - Core time management
- [[clock-python|Python Clock API]] - Python-specific details
- [[rate|Rate Specification]] - Fixed-rate execution with Clock

---

**Tip:** The ability to "fast-forward" time with SIM_TIME_TEST is what makes tests fast and deterministic!


