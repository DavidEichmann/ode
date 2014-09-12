module Constants where

--
-- Data
--

defaultDataDirectory = "/home/david/Documents/File Dump/git/ode/ODE_01/Data/Animation/"
defaultTestDataDirectory = defaultDataDirectory ++ "test/"


capturedDataScale :: Double
capturedDataScale = 0.01    -- captured data is in   cm = 0.01 m


--
-- Simulation
--

defaultTimeStep :: Double
defaultTimeStep = 1/100

-- physics

boneDensity :: Double
boneDensity = 1750

boneRadius :: Double
boneRadius = 0.1

gravityAcc :: Double
gravityAcc = 0.0000000001    -- !!!!!!!!!!! changed! in C++ and HASKELL

footWidth :: Double
footWidth = 0.2

boneRadiusDisplay :: Double
boneRadiusDisplay = boneRadius * 0.25

--
-- Model
--

-- distance from the floor counting as contact for the support pollygon
floorContactThreshold :: Double
floorContactThreshold = 0.001 * 5 -- 0.001 = 1 mm

-- maximum velocity of the foot during floor contact
floorContactVelocityThreshold :: Double
floorContactVelocityThreshold = 1 -- m/s

--
-- Motion preprocessing
--

-- When stopping foot sliding, this is how long it takes to blend the position of the ankle during floor contact back into the original motion
stepBlendTime :: Double
stepBlendTime = 0.5

-- This is the clearance between floor and foot that is considered large enough to not be changed by the
-- preprocessor.
safeFloorClearance :: Double
safeFloorClearance = 0.3

-- Floor clearance is exaggurated using this exponent. 1 is no exaduration, and 0 is descrete change from floor contact to safeFloorClearance
floorClearanceExponent :: Double
floorClearanceExponent = 0.4

--
-- Visualization
--

playbackSpeed :: Double
playbackSpeed = 1

frameRate :: Double
frameRate = 60

displayFDT :: Double
displayFDT = 1 / frameRate

defaultColorAlpha :: Double
defaultColorAlpha = 1

--
-- Feedback controller
--

defaultfeedBackControlUpdateInterval :: Double
defaultfeedBackControlUpdateInterval = 0

-- time it takes to blend into current motion
blendTime :: Double
blendTime = 0.1












