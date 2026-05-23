# DKR Events Message Format

DKR events are published as `std_msgs/String` on the `/dkr_events` topic
with JSON payloads. This avoids requiring a separate ament_cmake interface
package for custom message definitions.

## Event Types

### grant
```json
{
  "type": "grant",
  "timestamp": 1716123456.789,
  "robot": "warehouseRobot1",
  "resources": ["edge_5_6", "node_6"],
  "segment_idx": 2
}
```

### deny
```json
{
  "type": "deny",
  "timestamp": 1716123456.789,
  "robot": "warehouseRobot1",
  "resources_requested": ["edge_6_7", "node_7"],
  "blocking_robot": "warehouseRobot2",
  "reason": "resource_busy: edge_6_7 held by warehouseRobot2"
}
```

### release
```json
{
  "type": "release",
  "timestamp": 1716123456.789,
  "robot": "warehouseRobot1",
  "resources": ["node_5", "edge_5_6"]
}
```

### deadlock
```json
{
  "type": "deadlock",
  "timestamp": 1716123456.789,
  "cycle": ["warehouseRobot1", "warehouseRobot2", "warehouseRobot3"],
  "victim": "warehouseRobot3",
  "resolution": "yield"
}
```

### path_received
```json
{
  "type": "path_received",
  "timestamp": 1716123456.789,
  "robot": "warehouseRobot1",
  "task_id": "delivery.dispatch-abc123",
  "waypoint_count": 6,
  "total_resources": 11
}
```
