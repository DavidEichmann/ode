module Constants where

--
-- Simulation
--

defaultTimeStep :: Double
defaultTimeStep = 1/1000

-- physics

boneDensity :: Double
boneDensity = 350

boneRadius :: Double
boneRadius = 0.1

gravityAcc :: Double
gravityAcc = 9.8

footWidth :: Double
footWidth = 0.08

boneRadiusDisplay :: Double
boneRadiusDisplay = boneRadius * 0.25

--
-- Model
--

-- distance from the floor counting as contact for the support pollygon
floorContactThreshold :: Double
floorContactThreshold = 0.01



--
-- Visualization
--

playbackSpeed :: Double
playbackSpeed = 1/2

frameRate :: Double
frameRate = 60

defaultColorAlpha :: Double
defaultColorAlpha = 1

--
-- Feedback controller
--

defaultfeedBackControlUpdateInterval :: Double
defaultfeedBackControlUpdateInterval = 0.1

-- time it takes to blend into current motion
blendTime :: Double
blendTime = 0.1












