/**
 * @file test_terrain_classifier.cpp
 * @brief Unit tests for TerrainClassifier
 */

#include <gtest/gtest.h>
#include "tmr_costmap/terrain_classifier.hpp"

using namespace tmr_costmap;

class TerrainClassifierTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    config_.ground_height_tolerance = 0.05;
    config_.low_obstacle_height = 0.10;
    config_.medium_obstacle_height = 0.20;
    config_.high_obstacle_height = 0.30;
    config_.lethal_obstacle_height = 0.50;
    config_.max_traversable_slope = 15.0;
    config_.caution_slope = 25.0;
    config_.max_slope = 35.0;
    config_.cell_resolution = 0.05;
    config_.min_points_per_cell = 3;
    
    classifier_ = std::make_unique<TerrainClassifier>(config_);
  }

  TerrainClassifierConfig config_;
  std::unique_ptr<TerrainClassifier> classifier_;
};

TEST_F(TerrainClassifierTest, TestFreeTerrainClassification)
{
  // Create points at ground level
  std::vector<TerrainClassifier::PointT> points;
  for (int i = 0; i < 10; ++i) {
    TerrainClassifier::PointT p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0 + 0.01 * (i % 3);  // Small variation
    points.push_back(p);
  }

  TerrainCell cell = classifier_->classifyCell(points, 0.0, 0.0);

  EXPECT_EQ(cell.classification, TerrainClass::FREE);
  EXPECT_EQ(cell.cost, 0);
}

TEST_F(TerrainClassifierTest, TestLowObstacleClassification)
{
  // Create points at low obstacle height
  std::vector<TerrainClassifier::PointT> points;
  for (int i = 0; i < 10; ++i) {
    TerrainClassifier::PointT p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.12;  // Above low_obstacle_height
    points.push_back(p);
  }

  TerrainCell cell = classifier_->classifyCell(points, 0.0, 0.0);

  EXPECT_EQ(cell.classification, TerrainClass::LOW_COST);
  EXPECT_GT(cell.cost, 0);
  EXPECT_LT(cell.cost, 254);
}

TEST_F(TerrainClassifierTest, TestLethalObstacleClassification)
{
  // Create points at lethal height
  std::vector<TerrainClassifier::PointT> points;
  for (int i = 0; i < 10; ++i) {
    TerrainClassifier::PointT p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.6;  // Above lethal_obstacle_height
    points.push_back(p);
  }

  TerrainCell cell = classifier_->classifyCell(points, 0.0, 0.0);

  EXPECT_EQ(cell.classification, TerrainClass::LETHAL);
  EXPECT_EQ(cell.cost, 254);
}

TEST_F(TerrainClassifierTest, TestInsufficientPoints)
{
  // Create too few points
  std::vector<TerrainClassifier::PointT> points;
  points.push_back(TerrainClassifier::PointT());
  
  TerrainCell cell = classifier_->classifyCell(points, 0.0, 0.0);

  EXPECT_EQ(cell.classification, TerrainClass::UNKNOWN);
  EXPECT_EQ(cell.cost, 255);
}

TEST_F(TerrainClassifierTest, TestTerrainClassToCost)
{
  EXPECT_EQ(TerrainClassifier::terrainClassToCost(TerrainClass::FREE), 0);
  EXPECT_EQ(TerrainClassifier::terrainClassToCost(TerrainClass::LOW_COST), 50);
  EXPECT_EQ(TerrainClassifier::terrainClassToCost(TerrainClass::MEDIUM_COST), 128);
  EXPECT_EQ(TerrainClassifier::terrainClassToCost(TerrainClass::HIGH_COST), 200);
  EXPECT_EQ(TerrainClassifier::terrainClassToCost(TerrainClass::LETHAL), 254);
  EXPECT_EQ(TerrainClassifier::terrainClassToCost(TerrainClass::UNKNOWN), 255);
}

TEST_F(TerrainClassifierTest, TestHeightStatistics)
{
  std::vector<TerrainClassifier::PointT> points;
  
  for (float z = 0.0; z <= 0.1; z += 0.02) {
    TerrainClassifier::PointT p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = z;
    points.push_back(p);
  }

  TerrainCell cell = classifier_->classifyCell(points, 0.0, 0.0);

  EXPECT_NEAR(cell.min_height, 0.0, 0.01);
  EXPECT_NEAR(cell.max_height, 0.1, 0.01);
  EXPECT_GT(cell.mean_height, cell.min_height);
  EXPECT_LT(cell.mean_height, cell.max_height);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}