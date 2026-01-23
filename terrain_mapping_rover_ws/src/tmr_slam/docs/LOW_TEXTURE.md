# Low-Texture Terrain SLAM Guide

## The Challenge

Lunar/moon terrain presents significant challenges for visual SLAM:
- Uniform surface with minimal visual features
- Harsh shadows that change rapidly
- Dust and debris that obscure features
- Limited color variation (grayscale)

## Our Approach

The TMR uses a multi-pronged approach to handle low-texture environments:

### 1.  Aggressive Feature Detection

We use GFTT (Good Features to Track) with very low quality thresholds: 

```yaml
GFTT/QualityLevel: "0.0005"  # 10x lower than default
GFTT/MinDistance: "3"        # Closer features allowed
GFTT/BlockSize: "5"          # Larger context window
```

### 2. Depth-Assisted Matching

The Arducam ToF sensor provides depth information that helps even when visual features are sparse:

```yaml
Vis/DepthAsMask: "true"
Vis/MinDepth: "0.15"
Vis/MaxDepth: "3.5"
```

### 3. Strong Odometry

Our EKF fuses wheel odometry + IMU to provide reliable motion estimates: 
- Wheel odometry:  Good for distance
- IMU: Good for rotation
- Combined: Robust for SLAM

### 4. Proximity-Based Loop Closure

Instead of relying solely on visual similarity, we use spatial proximity: 

```yaml
RGBD/ProximityBySpace: "true"
RGBD/ProximityPathMaxNeighbors: "10"
```

## Recommended Configuration

Use the low-texture configuration: 

```bash
ros2 launch tmr_slam mapping.launch.py low_texture:=true
```

Or manually load: 

```bash
ros2 launch tmr_slam slam.launch.py config: =low_texture
```

## Testing Without Lunar Terrain

For development, create a low-texture test environment:

1. **White/Gray Room:** Cover surfaces with uniform gray material
2. **Low Lighting:** Reduce lighting to create flat illumination
3. **Minimal Objects:** Remove colorful or textured objects

## Enhancing Feature Detection

If natural features are insufficient, consider:

### ArUco Markers

Place ArUco markers at key locations:
- Entry/exit points
- Corners
- Every 2-3 meters along paths

```bash
# RTAB-MAP can detect ArUco markers
Aruco/Dictionary: "DICT_4X4_50"
```

### Artificial Texture

- Use LED lights to create shadows/highlights
- Place small rocks or objects as landmarks
- Create patterns with different colored dust

## Fallback:  Dead Reckoning

When visual SLAM completely fails:

1. EKF odometry continues to provide position
2. Map building pauses but navigation continues
3. Resume mapping when features return

## Metrics for Low-Texture Performance

| Metric | Acceptable | Target |
|--------|------------|--------|
| Features detected | >50 | >200 |
| Matching rate | >30% | >50% |
| Loop closure success | >50% | >80% |
| Position error per meter | <10cm | <5cm |

## Troubleshooting

### "Map going crazy"

The map is jumping or distorting:
1. Increase `Rtabmap/LoopThr` to 0.15
2. Increase `Vis/MinInliers` to 20
3. Check for repetitive patterns causing false matches

### "Never getting loop closures"

Robot returns to start but no correction: 
1. Decrease `Rtabmap/LoopThr` to 0.08
2. Decrease `Vis/MinInliers` to 10
3. Add visual markers at key points

### "Map drifts continuously"

Position error accumulates:
1. Check wheel odometry calibration
2. Check IMU calibration
3. Ensure depth data is being used
4. Consider reducing travel speed
