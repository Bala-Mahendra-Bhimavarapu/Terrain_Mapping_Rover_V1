# RTAB-MAP Tuning Guide

## Overview

This guide explains how to tune RTAB-MAP SLAM for the Terrain Mapping Rover, with special focus on low-texture lunar terrain environments.

## Key Parameters

### Feature Detection

RTAB-MAP uses visual features for loop closure detection.  The choice of feature detector is critical:

| Detector | Best For | Speed | Notes |
|----------|----------|-------|-------|
| GFTT/ORB | Low texture | Fast | Recommended for TMR |
| SURF | General | Medium | Good accuracy |
| SIFT | High detail | Slow | Best accuracy |
| ORB | Speed | Very Fast | May miss features |

**Configuration:**
```yaml
Vis/FeatureType: "8"  # GFTT/ORB
Vis/MaxFeatures: "1000"  # More for low texture
```

### GFTT Parameters (Good Features to Track)

For low-texture terrain, GFTT parameters are critical:

```yaml
# Lower quality = more features detected
GFTT/QualityLevel: "0.001"  # Default: 0.01

# Smaller distance = more features
GFTT/MinDistance: "5"  # Default: 7

# Larger block = more context
GFTT/BlockSize: "5"  # Default: 3
```

### Loop Closure Detection

Loop closure is essential for correcting odometry drift:

```yaml
# Lower threshold = more potential matches (may increase false positives)
Rtabmap/LoopThr: "0.11"  # Default: 0.11

# Minimum inliers for valid match
Vis/MinInliers: "15"  # Lower for hard environments

# RANSAC iterations
Vis/Iterations: "300"  # More for robustness
```

### Memory Management

Control how RTAB-MAP manages map data:

```yaml
# Working memory size
Mem/STMSize: "30"  # Nodes in short-term memory

# Rehearsal threshold (how similar before merging)
Mem/RehearsalSimilarity: "0.6"  # Lower = keep more diverse nodes
```

## Tuning Procedure

### Step 1: Verify Sensor Data

Before tuning SLAM, ensure sensors are working: 

```bash
# Check RGB image
ros2 topic hz /camera/image_raw

# Check depth
ros2 topic hz /tof/depth/image_raw

# Check odometry
ros2 topic hz /odometry/filtered
```

### Step 2: Start with Default Config

```bash
ros2 launch tmr_slam slam. launch. py
```

### Step 3: Monitor Feature Detection

Watch the RTAB-MAP visualization to see: 
- Number of features detected (should be 200+)
- Feature distribution (should cover the image)
- Matching success rate

### Step 4: Adjust for Your Environment

**Indoor (lots of features):**
```yaml
GFTT/QualityLevel: "0.01"
Vis/MaxFeatures: "500"
Vis/MinInliers: "20"
```

**Outdoor/Low texture:**
```yaml
GFTT/QualityLevel:  "0.001"
Vis/MaxFeatures: "2000"
Vis/MinInliers: "10"
```

### Step 5: Test Loop Closure

Drive the robot in a loop and return to the starting point.  Watch for:
- Loop closure detection
- Map correction after loop closure

## Common Problems

### Problem:  No Features Detected

**Symptoms:** Very few or no visual features in RTAB-MAP visualization

**Solutions:**
1. Lower `GFTT/QualityLevel` (e.g., 0.0005)
2. Increase `Vis/MaxFeatures` (e.g., 2000)
3. Check camera exposure/lighting

### Problem: False Loop Closures

**Symptoms:** Map jumps or becomes distorted

**Solutions:**
1. Increase `Rtabmap/LoopThr` (e.g., 0.15)
2. Increase `Vis/MinInliers` (e.g., 25)
3. Use depth validation

### Problem: No Loop Closures

**Symptoms:** Map drifts, no corrections when returning to visited areas

**Solutions:**
1. Lower `Rtabmap/LoopThr` (e.g., 0.08)
2. Lower `Vis/MinInliers` (e.g., 10)
3. Ensure enough features are detected

### Problem: High CPU Usage

**Solutions:**
1. Reduce `Vis/MaxFeatures`
2. Increase image decimation
3. Lower detection rate:  `Rtabmap/DetectionRate:  "1. 0"`

## Low-Texture Terrain Tips

For moon-like terrain with minimal visual features:

1. **Maximize Feature Extraction:**
   ```yaml
   GFTT/QualityLevel: "0.0005"
   Vis/MaxFeatures: "2000"
   ```

2. **Use Depth Information:**
   ```yaml
   Vis/DepthAsMask: "true"
   ```

3. **Relaxed Matching:**
   ```yaml
   Vis/MinInliers: "10"
   Vis/MaxDepthError: "0.2"
   ```

4. **Proximity Detection:**
   ```yaml
   RGBD/ProximityBySpace: "true"
   ```

5. **Consider Adding Markers:**
   - Place visual markers (ArUco) for reliable loop closures
   - Create artificial texture with lights/shadows

## Performance Metrics

Monitor these metrics during operation:

| Metric | Good | Warning | Action |
|--------|------|---------|--------|
| Features/image | >200 | <100 | Lower quality threshold |
| Loop closures/min | >0.5 | 0 | Check parameters |
| CPU usage | <70% | >90% | Reduce features/rate |
| Map drift | <5cm/m | >10cm/m | Tune loop closure |

## Configuration Files

- `rtabmap_params.yaml` - Default configuration
- `rtabmap_low_texture.yaml` - Optimized for lunar terrain
- `rtabmap_indoor.yaml` - For indoor testing