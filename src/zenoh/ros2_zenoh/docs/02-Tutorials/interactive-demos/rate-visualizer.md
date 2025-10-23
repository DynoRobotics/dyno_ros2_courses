# Interactive Rate Visualizer

**Live demonstration of fixed-rate execution patterns**

This interactive demo shows how `Rate` and `fixed_rate_loop()` maintain consistent execution rates even when processing time varies.

---

## Try It: Rate Control Simulation

```dataviewjs
const app = dv.container;

// Create UI
app.innerHTML = `
<div style="padding: 1.5em; border: 2px solid #2196F3; border-radius: 8px; background: #f8f9fa;">
    <h3 style="margin-top: 0; color: #1976D2;">🔄 Fixed-Rate Loop Simulator</h3>
    
    <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 1em; margin: 1em 0;">
        <div>
            <label style="display: block; margin-bottom: 0.5em;">
                <strong>Target Rate:</strong>
                <input type="range" id="target-rate" min="1" max="50" value="10" style="width: 100%;">
                <span id="rate-val" style="color: #2196F3; font-weight: bold;">10</span> Hz
            </label>
        </div>
        
        <div>
            <label style="display: block; margin-bottom: 0.5em;">
                <strong>Processing Time:</strong>
                <input type="range" id="proc-time" min="0" max="150" value="50" style="width: 100%;">
                <span id="proc-val" style="color: #FF9800; font-weight: bold;">50</span> ms
            </label>
        </div>
    </div>
    
    <div style="text-align: center; margin: 1em 0;">
        <button id="run-btn" style="padding: 10px 24px; font-size: 16px; background: #4CAF50; color: white; border: none; border-radius: 4px; cursor: pointer;">
            ▶️ Run Simulation
        </button>
        <button id="stop-btn" style="padding: 10px 24px; font-size: 16px; background: #F44336; color: white; border: none; border-radius: 4px; cursor: pointer; margin-left: 10px;" disabled>
            ⏹️ Stop
        </button>
    </div>
    
    <canvas id="viz" width="700" height="350" style="border: 1px solid #ddd; margin: 1em 0; background: white; display: block; width: 100%;"></canvas>
    
    <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 1em; margin-top: 1em;">
        <div id="stats" style="font-family: 'Courier New', monospace; font-size: 13px; background: #263238; color: #4CAF50; padding: 1em; border-radius: 4px;"></div>
        <div id="legend" style="padding: 1em; background: #FAFAFA; border-radius: 4px;">
            <div style="margin-bottom: 0.5em;">
                <span style="display: inline-block; width: 20px; height: 3px; background: #2196F3; vertical-align: middle;"></span>
                <span style="margin-left: 8px;">Expected Tick Time</span>
            </div>
            <div style="margin-bottom: 0.5em;">
                <span style="display: inline-block; width: 20px; height: 20px; background: #4CAF50; vertical-align: middle;"></span>
                <span style="margin-left: 8px;">On-Time Execution</span>
            </div>
            <div style="margin-bottom: 0.5em;">
                <span style="display: inline-block; width: 20px; height: 20px; background: #F44336; vertical-align: middle;"></span>
                <span style="margin-left: 8px;">Deadline Overrun</span>
            </div>
            <div style="margin-top: 1em; padding-top: 1em; border-top: 1px solid #ddd; font-size: 11px; color: #666;">
                <strong>How to interpret:</strong><br>
                • Blue lines show expected timing<br>
                • Green bars = processing fits within period<br>
                • Red bars = processing exceeds period
            </div>
        </div>
    </div>
</div>
`;

const canvas = app.querySelector('#viz');
const ctx = canvas.getContext('2d');
const targetRateSlider = app.querySelector('#target-rate');
const procTimeSlider = app.querySelector('#proc-time');
const rateVal = app.querySelector('#rate-val');
const procVal = app.querySelector('#proc-val');
const runBtn = app.querySelector('#run-btn');
const stopBtn = app.querySelector('#stop-btn');
const stats = app.querySelector('#stats');

let isRunning = false;
let shouldStop = false;

// Update labels
targetRateSlider.addEventListener('input', (e) => {
    rateVal.textContent = e.target.value;
});

procTimeSlider.addEventListener('input', (e) => {
    procVal.textContent = e.target.value;
});

// Draw initial state
function drawInitialState() {
    ctx.clearRect(0, 0, 700, 350);
    ctx.fillStyle = '#f5f5f5';
    ctx.fillRect(0, 0, 700, 350);
    
    ctx.fillStyle = '#999';
    ctx.font = '18px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('Click "Run Simulation" to start', 350, 175);
    
    ctx.font = '14px sans-serif';
    ctx.fillText('Adjust rate and processing time, then run', 350, 200);
}

drawInitialState();

// Simulation
async function runSimulation() {
    if (isRunning) return;
    
    isRunning = true;
    shouldStop = false;
    runBtn.disabled = true;
    stopBtn.disabled = false;
    targetRateSlider.disabled = true;
    procTimeSlider.disabled = true;
    
    const targetHz = Number(targetRateSlider.value);
    const procTime = Number(procTimeSlider.value) / 1000;  // ms to s
    const period = 1.0 / targetHz;
    
    // Clear canvas
    ctx.clearRect(0, 0, 700, 350);
    
    // Draw grid
    ctx.strokeStyle = '#e0e0e0';
    ctx.lineWidth = 1;
    for (let i = 0; i <= 10; i++) {
        const y = 50 + i * 25;
        ctx.beginPath();
        ctx.moveTo(50, y);
        ctx.lineTo(650, y);
        ctx.stroke();
    }
    
    // Draw baseline
    ctx.strokeStyle = '#666';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(50, 300);
    ctx.lineTo(650, 300);
    ctx.stroke();
    
    // Labels
    ctx.fillStyle = '#333';
    ctx.font = '12px sans-serif';
    ctx.textAlign = 'right';
    ctx.fillText('Time', 45, 305);
    
    // Simulate
    let currentTime = 0;
    let iteration = 0;
    let overruns = 0;
    let totalOverrun = 0;
    
    const maxIterations = 50;
    const barWidth = 10;
    const startX = 60;
    const spacing = (650 - startX) / maxIterations;
    
    for (let i = 0; i < maxIterations; i++) {
        if (shouldStop) break;
        
        iteration++;
        const expectedTime = iteration * period;
        
        // Simulate processing
        currentTime += procTime;
        
        // Check if overrun
        const overrun = Math.max(0, currentTime - expectedTime);
        const missedDeadline = overrun > 0;
        
        if (missedDeadline) {
            overruns++;
            totalOverrun += overrun;
        }
        
        // Draw tick
        const x = startX + i * spacing;
        const baseY = 300;
        
        // Expected time (blue line)
        ctx.strokeStyle = '#2196F3';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(x, baseY);
        ctx.lineTo(x, baseY - 80);
        ctx.stroke();
        
        // Actual execution (bar)
        ctx.fillStyle = missedDeadline ? '#F44336' : '#4CAF50';
        const barHeight = 60 + (missedDeadline ? Math.min(overrun * 200, 100) : 0);
        ctx.fillRect(x - barWidth/2, baseY, barWidth, -Math.min(barHeight, 150));
        
        // Overrun indicator
        if (missedDeadline) {
            ctx.fillStyle = '#FF5722';
            ctx.font = 'bold 10px sans-serif';
            ctx.textAlign = 'center';
            ctx.fillText('!', x, baseY - Math.min(barHeight, 150) - 5);
        }
        
        // Sleep until next period (skip-if-behind behavior)
        if (!missedDeadline) {
            currentTime = expectedTime + period;
        } else {
            // Behind schedule - just advance to next expected tick
            currentTime = Math.max(currentTime, expectedTime) + period;
        }
        
        // Update stats
        const avgOverrun = overruns > 0 ? (totalOverrun / overruns * 1000).toFixed(2) : 0;
        const efficiency = ((iteration - overruns) / iteration * 100).toFixed(1);
        
        stats.innerHTML = `
> Iteration: ${iteration}/${maxIterations}
> Target Rate: ${targetHz} Hz (${(period * 1000).toFixed(1)}ms period)
> 
> On Time: ${iteration - overruns} ✓
> Overruns: ${overruns} ⚠️
> Efficiency: ${efficiency}%
> 
> Avg Overrun: ${avgOverrun}ms
> Current Time: ${currentTime.toFixed(3)}s
        `.trim();
        
        await new Promise(r => setTimeout(r, 80));
    }
    
    // Final stats
    if (!shouldStop) {
        const efficiency = ((iteration - overruns) / iteration * 100).toFixed(1);
        stats.innerHTML += `\n\n✅ Simulation Complete!\n📊 Final Efficiency: ${efficiency}%`;
    } else {
        stats.innerHTML += `\n\n⏹️ Simulation Stopped`;
    }
    
    isRunning = false;
    runBtn.disabled = false;
    stopBtn.disabled = true;
    targetRateSlider.disabled = false;
    procTimeSlider.disabled = false;
}

runBtn.addEventListener('click', runSimulation);
stopBtn.addEventListener('click', () => {
    shouldStop = true;
    stopBtn.disabled = true;
});
```

---

## What This Demonstrates

### Rate Control Behavior

1. **On-Time Execution** (Green)
   - Processing time < period
   - No deadline misses
   - Consistent timing

2. **Overrun Detection** (Red)
   - Processing time > period
   - Deadline missed
   - Skip-if-behind behavior

3. **Efficiency Metrics**
   - Percentage of on-time iterations
   - Average overrun duration
   - Real-time statistics

### Key Observations

**Try these experiments:**

1. **Fast Rate, Fast Processing**
   - Rate: 50 Hz (20ms period)
   - Processing: 10ms
   - **Result**: 100% efficiency ✅

2. **Moderate Rate, Variable Processing**
   - Rate: 10 Hz (100ms period)
   - Processing: 50ms
   - **Result**: High efficiency, no overruns ✅

3. **Slow Processing**
   - Rate: 10 Hz (100ms period)
   - Processing: 150ms
   - **Result**: Frequent overruns, reduced efficiency ⚠️

4. **Edge Case**
   - Rate: 10 Hz (100ms period)
   - Processing: 100ms (exactly at limit)
   - **Result**: Watch for occasional overruns due to scheduling overhead

---

## Code Equivalent

This visualization shows the behavior of:

```python
from ros2_zenoh_python import fixed_rate_loop, Clock, TimeMode

async def control_loop():
    clock = Clock(TimeMode.WALL_TIME)
    
    async for tick in fixed_rate_loop(10.0, clock):  # 10 Hz
        # Your processing here
        await process_data()
        
        # Check deadline
        if tick.missed_deadline:
            print(f"Overrun: {tick.overrun * 1000:.1f}ms")
        
        if tick.iteration >= 100:
            break
```

---

## Learn More

- [[rate|Rate Specification]] - Core rate control spec
- [[rate-python|Python Rate API]] - Python-specific details
- [[clock|Clock Specification]] - Time management

---

**Note**: This is a simplified simulation. Real-world behavior depends on OS scheduling, system load, and other factors.


