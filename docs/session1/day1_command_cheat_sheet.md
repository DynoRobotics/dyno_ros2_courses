# Session 1 - Useful commands

## Enable X-forwarding from container (run outside docker)
```bash
xhost +local:docker
```

## Run zenoh router
```bash
just zenohd
```

## Start basic simulation
```bash
just l_turtle_2
```

## Teleoperate turtle
```bash
just teleop
```

## Start web interface for sending actions
```bash
just web
```