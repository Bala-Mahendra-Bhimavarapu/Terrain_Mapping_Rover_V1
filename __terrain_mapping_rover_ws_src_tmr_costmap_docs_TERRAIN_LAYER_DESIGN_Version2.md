# ToF Terrain Layer Design

## Overview

The ToF Terrain Layer is a custom Nav2 costmap layer plugin that processes point cloud data from the Arducam ToF camera to classify terrain and update the costmap for navigation.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     ToF Terrain Layer                        │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────┐    ┌────────────────┐    ┌─────────────┐ │
│  │ PointCloud2  │───▶│ PointCloud     │───▶│  Terrain    │ │
│  │ Subscriber   │    │ Processor      │    │  Classifier │ │
│  └──────────────┘    └────────────────┘    └─────────────┘ │
│                              │                     │        │
│                              ▼                     ▼        │
│                      ┌─────────────┐       ┌─────────────┐ │
│                      │ Filtered    │       │  Terrain    │ │
│                      │ Cloud       │       │  Cells      │ │
│                      └─────────────┘       └──────���──────┘ │
│                                                    │        │
│                                                    ▼        │
│                                            ┌─────────────┐ │
│                                            │  Costmap    │ │
│                                            │  Update     │ │
│                                            └─────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

## Components

### 1. Point Cloud Processor

Preprocesses the raw ToF point cloud:
- **Transform**: Converts to robot base frame
- **Voxel Filter**: Downsamples for efficiency
- **Range Filter**: Removes points too close/far
- **Height Filter**: Removes points above/below thresholds
- **Outlier Removal**: Removes noise

### 2. Terrain Classifier

Analyzes the processed point cloud and classifies terrain:

#### Classification Criteria

| Class | Height (m) | Slope (°) | Cost |
|-------|-----------|-----------|------|
| FREE | < 0.08 | < 15 | 0 |
| LOW_COST | 0.08 - 0.15 | 15 - 20 | 50 |
| MEDIUM_COST | 0.15 - 0.25 | 20 - 25 | 128 |
| HIGH_COST | 0.25 - 0.40 | 25 - 35 | 200 |
| LETHAL | > 0.40 | > 35 | 254 |

#### Algorithm

1. **Grid Creation**: Divide area into cells
2. **Point Binning**: Assign points to cells
3. **Per-Cell Analysis**:
   - Calculate min/max/mean height
   - Estimate slope from height variance
   - Calculate roughness (std dev)
4. **Classification**: Apply thresholds
5. **Cost Assignment**: Map class to Nav2 cost

### 3. Costmap Update

Updates the Nav2 costmap with classified terrain:
- Uses `updateBounds()` to mark affected region
- Uses `updateCosts()` to write cost values
- Respects existing obstacles (max cost wins)

## Data Flow

```
/tof/points (sensor_msgs/PointCloud2)
    │
    ▼
Transform to base_link frame
    │
    ▼
Apply filters (voxel, range, height, outlier)
    │
    ▼
Bin points into grid cells
    │
    ▼
Classify each cell (height, slope, roughness)
    │
    ▼
Generate TerrainCell list
    │
    ▼
Update costmap costs
    │
    ▼
Nav2 uses costmap for planning
```

## Configuration

### Key Parameters

```yaml
# Height thresholds
low_obstacle_height: 0.08      # Start of low-cost zone
medium_obstacle_height: 0.15   # Start of medium-cost zone
high_obstacle_height: 0.25     # Start of high-cost zone
lethal_obstacle_height: 0.40   # Lethal (non-traversable)

# Slope thresholds (degrees)
max_traversable_slope: 15.0    # Free traversal
caution_slope: 25.0            # Reduced speed
max_slope: 35.0                # Maximum traversable

# Processing
cell_resolution: 0.05          # Grid cell size
min_points_per_cell: 3         # Minimum for valid cell
```

## Integration with Nav2

The layer integrates as a standard Nav2 costmap layer:

```yaml
local_costmap:
  plugins: ["tof_terrain_layer", "inflation_layer"]
  
  tof_terrain_layer:
    plugin: "tmr_costmap/ToFTerrainLayer"
    enabled: true
    point_cloud_topic: "/tof/points"
```

## Performance Considerations

1. **Update Rate**: Process at ToF camera rate (~15 Hz)
2. **Voxel Size**: Balance detail vs. speed
3. **Cell Resolution**: Match costmap resolution
4. **Observation Persistence**: Clear old data appropriately

## Lunar Terrain Adaptations

For NASA HUNCH lunar terrain simulation:

1. **Lower obstacle thresholds**: Lunar rocks may be smaller
2. **Slope sensitivity**: Moon gravity affects traversability
3. **Roughness handling**: Regolith texture consideration
4. **Shadow handling**: ToF works in shadows (advantage)