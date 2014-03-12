{-# OPTIONS_GHC -F -pgmF htfpp #-}
{-# LANGUAGE BangPatterns #-}

module MotionData where

import Control.Applicative (liftA2)
import Data.List
import Data.Char
import Data.TreeF
import Text.Parsec
import Text.Parsec.Token
import Text.Parsec.Language
import Linear
import Util
import Constants
import Test.Framework

prop_a = 1 == 1
test_empty = assertEqual ([] :: [Int]) ([])

data Channel = Xpos | Ypos | Zpos | Xrot | Yrot | Zrot deriving (Eq, Show)
posChans :: [Channel]
posChans = [Xpos,Ypos,Zpos]
rotChans :: [Channel]
rotChans = [Xrot,Yrot,Zrot]

--type JointF = TreeF Joint

data Joint = Joint {
                name       :: String,
                offset     :: Vec3,
                rotationL  :: Quat,
                channels   :: [Channel],
                linearVel  :: Vec3,
                linearAcc  :: Vec3,
                angularVel :: Vec3,
                angularAcc :: Vec3
        } deriving Show
                
-- this data (mass/inertiaM) refers to the bone between this and parent joint

density :: JointF -> Double
density jF
        | hasParent jF   = 350
        | otherwise     = 0
        
        
mass :: JointF -> Double
mass = fst . massInertiaM
        
inertiaM :: JointF -> M3
inertiaM = snd . massInertiaM
        
-- inertia matrix is computed as that of a capsule represented by the offset vector
-- as the root joint does not represent a bone, it has no valid inertia matrix, so zero is used  
massInertiaM :: JointF -> (Double, M3)
massInertiaM jF
        | hasParent jF   = getCapsulMassInertiaM (density jF) boneRadius (offset $- jF)
        | otherwise     = (0,zero)
        
getCapsulMassInertiaM :: Double -> Double -> Vec3 -> (Double,M3)
getCapsulMassInertiaM d r boneVec = --(mass, rotM !*! xm !*! rotInvM) where
    (mass, rotM !*! xm !*! rotInvM) where
        (mass, xm) = getXCapsulMassInertiaM d r (norm boneVec)
        rot     = xToDirQuat boneVec
        rotInv  = conjugate rot
        rotM    = fromQuaternion rot
        rotInvM = fromQuaternion rotInv

getXCapsulMassInertiaM :: Double -> Double -> Double -> (Double,M3)
getXCapsulMassInertiaM d r l = (m, im) where
    r2 = r ** 2
    l2 = l ** 2
    m1 = pi * l * r2 * d
    m = m1 {- +m2
    
    For now just use a cylinder 
    m2 = (4/3) * pi * r2 * r * d
    ia = m1 * (0.25 * r2 + (1/12) * l2) +
         m2 * (0.4  * r2 + 0.375  * r * l + 0.25 * l2)
    ib = ((m1 * 0.5) + (m2 * 0.4)) * r2;
    
    im = V3 (V3  ib  0   0)
            (V3  0   ia  0)
            (V3  0   0   ia)
            -}
            
    iX  = 0.5 * m1 * r2
    iZY = (1/12) * m1 * ((3 * r2) + (l2))  
    im = V3 (V3  iX  0   0)
            (V3  0   iZY  0)
            (V3  0   0   iZY)


getTotalMass :: JointF -> Double
getTotalMass rf = treeFoldNR (\m jf -> m + (mass jf)) 0 rf

getPosTotalCom :: JointF -> Vec3
getPosTotalCom rf = (treeFoldNR (\p jf -> p + (getPosCom jf ^* ((mass jf) / (m_tot)))) zero rf) where
    m_tot = getTotalMass rf
    
getLinearVel :: JointF -> Vec3
getLinearVel f = linearVel $- f

getLinearAcc :: JointF -> Vec3
getLinearAcc f = linearAcc $- f

getAngularVel :: JointF -> Vec3
getAngularVel f = angularVel $- f

getAngularAcc :: JointF -> Vec3
getAngularAcc f = angularAcc $- f

getLinearMomentum :: JointF -> Vec3
getLinearMomentum jf = (getLinearVel jf) ^* (mass jf)

getLinearMomentum' :: JointF -> Vec3
getLinearMomentum' jf = (getLinearAcc jf) ^* (mass jf)

getTotalLinearMomentum :: JointF -> Vec3
getTotalLinearMomentum rf = treeFoldNR (\p jf -> p + (getLinearMomentum jf)) zero rf

getTotalLinearMomentum' :: JointF -> Vec3
getTotalLinearMomentum' rf = treeFoldNR (\p jf -> p + (getLinearMomentum' jf)) zero rf

--this is about the center of mass, but using the global coordinate system
getAngularMomentum :: JointF -> Vec3
getAngularMomentum rf = rot `rotate` ((inertiaM rf) !* (rotInv `rotate` (getAngularVel rf))) where
    rot = getRot rf
    rotInv = conjugate rot

getAngularMomentum' :: JointF -> Vec3
getAngularMomentum' rf = rot `rotate` ((inertiaM rf) !* (rotInv `rotate` (getAngularAcc rf))) where
    rot = getRot rf
    rotInv = conjugate rot

getTotalAngularMomentum :: JointF -> Vec3
getTotalAngularMomentum rf = treeFoldNR (\h jf -> h + (((getPosCom jf) `cross` (getLinearMomentum jf)) + (getAngularMomentum jf))) zero rf

getTotalAngularMomentum' :: JointF -> Vec3
getTotalAngularMomentum' rf = treeFoldNR sumH zero rf where
    sumH h jf = h +
        ((getLinearVel jf) `cross` (getLinearMomentum  jf)) +
        ((getPosCom jf)    `cross` (getLinearMomentum' jf)) + 
        ((getAngularMomentum' jf)) +
        ((getAngularVel jf) `cross` (getAngularMomentum jf))
    
getZMP :: JointF -> Vec3
getZMP jf = zmp where
    m_tot = getTotalMass jf
    g = gravityAcc
    (V3 comX _ comZ) = getPosTotalCom jf
    (V3 h'X _ h'Z) = getTotalAngularMomentum' jf
    (V3 _ p'Y _) = getTotalLinearMomentum' jf

    -- assume that ZMP is on the floor (y component is 0)
    -- to preserve right-handedness we must swap (ZMP paper axis -> my code axis): z -> y, y -> x, x -> y)
    zmp = V3
        (((m_tot * g * comX) + h'Z) / ((m_tot * g) + p'Y))
        0
        (((m_tot * g * comZ) - h'X) / ((m_tot * g) + p'Y))

nullJoint :: Joint
nullJoint = Joint {
        name = "",
        offset = zero,
        rotationL = identity,
        channels = [],
        linearVel = zero,
        linearAcc = zero,
        angularVel = zero,
        angularAcc = zero
}

type JointTree = (Tree Joint)
type JointF = (TreeF Joint)

numChannels :: Joint -> Int
numChannels = length . channels

hasPosChan :: Joint -> Bool
hasPosChan j = not $ null (intersect posChans (channels j))
hasRotChan :: Joint -> Bool
hasRotChan j = not $ null (intersect rotChans (channels j))

type Frame = JointF
data MotionData = MotionData {
        frames        :: [Frame],
        frameTime     :: Double,
        baseSkeleton  :: JointF
} deriving Show

scale :: Double -> MotionData -> MotionData
scale s md = md{
                frames = map (jointScale s) (frames md),
                baseSkeleton = jointScale s (baseSkeleton md)
        }
        
translate :: Vec3 -> MotionData -> MotionData
translate t md = md{
                frames = map (jointTranslate t) (frames md),
                baseSkeleton = jointTranslate t (baseSkeleton md)
        }
        
jointScale :: Double -> JointF -> JointF
jointScale s = treeMap (map1 (\j@Joint{offset=o} -> j{offset=o ^* s}))
        
jointTranslate :: Vec3 -> JointF -> JointF
jointTranslate t = map1 (\r@Joint{offset=o} -> r{offset=o+t})

fromChanVals :: JointF -> [Double] -> JointF
fromChanVals jf cv = (\(d,r) -> if d == [] then r else error $ "chan vals left over: " ++ (show d)) $ treeMapCon applyChans cv jf where
         applyChans cv jf = (cv', set (sk{ offset = offset', rotationL = rotationL'}) jf) where
                sk = view jf 
                baseOffset = offset sk
                baseRot    = rotationL sk
                (offset', rotationL', cv') = consume baseOffset baseRot (channels sk) cv
                consume ost rot [] vs = (ost, rot, vs)
                consume _ _ _ [] = error("Not enough values to fill channels")
                consume (V3 _ py pz) rot (Xpos:cs) (v:vs) = consume (V3 v py pz) rot cs vs
                consume (V3 px _ pz) rot (Ypos:cs) (v:vs) = consume (V3 px v pz) rot cs vs
                consume (V3 px py _) rot (Zpos:cs) (v:vs) = consume (V3 px py v) rot cs vs
                
                
               {- consume pos rot (Yrot:Xrot:Zrot:cs) (y:x:z:vs) = consume pos (
                        rot *
                        (axisAngle (unitY) (degreeToRadian y)) *
                        (axisAngle (unitX) (degreeToRadian x)) *
                        (axisAngle (unitZ) (degreeToRadian z))
                    ) cs vs -}
                
                consume pos rot (Xrot:cs) (v:vs) = consume pos (rot * (axisAngle (unitX) (degreeToRadian v))) cs vs
                consume pos rot (Yrot:cs) (v:vs) = consume pos (rot * (axisAngle (unitY) (degreeToRadian v))) cs vs
                consume pos rot (Zrot:cs) (v:vs) = consume pos (rot * (axisAngle (unitZ) (degreeToRadian v))) cs vs
                         

parseBVH :: String -> Integer -> IO (MotionData)
parseBVH filePath ppFilterWindow = do
        bvh <- raw_parseBVH filePath
        return $ calculateDynamics $ (if ppFilterWindow > 0 then preProcess ppFilterWindow else id) bvh --{frames = replicate 100 ((frames bvh) !! 10)}

-- currently this just uses moving average for joint angles and positions 
preProcess :: Integer -> MotionData -> MotionData
preProcess windowSize md = mdF where
    n = fromInteger windowSize
    mdF = md{
             frames = map avg (windows (frames md))
        }
    windows fs
        | length lfsn == n  = lfsn : windows (tail fs)
        | otherwise         = []
        where
            lfsn = take n fs
    avg fs = fmap avgJ (foldl1 (treeZipWith (\a b -> (view a) ++ (view b))) (map (fmap (:[])) fs))
    avgJ :: [Joint] -> Joint
    avgJ js = (head js) {
        offset   = avgPos,
        rotationL = avgRotL
    }  where
        avgPos = (foldl1 (+) (map offset js)) ^/ (fromIntegral n)
        avgRotL = avgRot (map rotationL js)

calculateDynamics :: MotionData -> MotionData
calculateDynamics md = md{frames = fva} where
    f = frames md
    fv =  map (uncurry $ treeZipWith calcVels) (zip f  (tail f))
    fva = map (uncurry $ treeZipWith calcAccs) (zip fv (tail fv))
    
    
    dt = frameTime md 
    
    calcVels :: JointF -> JointF -> Joint
    calcVels a b
        | hasParent a  = (view a){
                                linearVel  = ((getPosCom b) - (getPosCom a))  ^/  dt,
                                angularVel = toAngularVel (getRot a) (getRot b)  dt
                            }
        | otherwise     = (view a){
                                linearVel  = zero,
                                angularVel = zero
                            }
    
    calcAccs :: JointF -> JointF -> Joint
    calcAccs a b
        | hasParent a  = (view a){
                                linearAcc  = ((getLinearVel b)  - (getLinearVel a))   ^/  dt,
                                angularAcc = ((getAngularVel b) - (getAngularVel a))  ^/  dt
                            }
        | otherwise     = (view a){
                                linearAcc  = zero,
                                angularAcc = zero
                            }

toAngularVel :: Quat -> Quat -> Double -> Vec3
toAngularVel qi qf dt = if angleHalf == 0 then zero else ((2 * angleHalf) *^ rotAxis) ^/ dt where
                        Quaternion qlnW qlnV = qf * (conjugate qi)
                        angleHalf = acos $ max (-1) (min qlnW 1)
                        rotAxis = qlnV ^/ (sin angleHalf)

{-------------------------

        HIGH LEVEL
        FUNCTIONS

--------------------------}

getRot :: JointF -> Quat
getRot j = case parent j of
        Nothing  ->  rotationL $- j
        Just p   ->  (getRot p) * (rotationL $- j)
        
getPosEnd :: JointF -> Vec3
getPosEnd j = getPosEnd' (parent j) where
        getPosEnd' Nothing = offset $- j
        getPosEnd' (Just p) = (getPosEnd p) + ((getRot p) `rotate` (offset $- j))

getPosStart :: JointF -> Vec3
getPosStart j = maybe zero getPosEnd (parent j)

getPosCom :: JointF -> Vec3
getPosCom jf = case parent jf of
        Nothing  ->  getPosEnd jf
        Just _   ->  ((getPosStart jf) + (getPosEnd jf)) / 2 where


-- get the (min, max) coordinates of the system
frameBoundingBox :: Frame -> (Vec3, Vec3)
frameBoundingBox f = treeFoldNR minMax initMinMax f where
        initMinMax = let o = offset $- f in (o, o)
        minMax :: (Vec3, Vec3) -> JointF -> (Vec3, Vec3)
        minMax (cMin,cMax) v = let vPos = getPosEnd v in ((liftA2 min) cMin vPos, (liftA2 max) cMax vPos)



scaleAndTranslate :: Double -> Int -> MotionData -> MotionData
scaleAndTranslate targetHeight sampleFrameIndex md = calculateDynamics $ mdTS where
    bb = frameBoundingBox $ (frames md)!!sampleFrameIndex
    bbDiag = uncurry (+) bb
    bbC@(V3 _ cHeight _) = bbDiag / 2
    targetC = (V3 0 (cHeight + boneRadius) 0)
    translation = targetC - bbC
    scaleFactor = let V3 _ height _ = bbDiag in targetHeight / height
    
    mdT  = translate translation md
    mdTS = scale scaleFactor mdT

    
-- Parsing code

hsToken = makeTokenParser haskellStyle

wordP = do
        spaces
        many1 (satisfy (not. isSpace))

floatP = do
        num <- wordP
        return (read num :: Double)
        
intP = do
        num <- wordP
        return (read num :: Int)

vec3 = do
        x <- floatP
        y <- floatP
        z <- floatP
        return (V3 x y z)

hierarchy = do
        string "HIERARCHY"
        spaces
        skel <- fmap head joints
        return skel
        where
                joints = many (joint <|> endSite)
                joint = do
                        string "ROOT" <|> string "JOINT"
                        spaces
                        name <- wordP
                        spaces
                        char '{'
                        spaces
                        string "OFFSET"
                        spaces
                        offset <- vec3
                        spaces
                        string "CHANNELS"
                        spaces
                        numC <- intP
                        spaces
                        channels <- channels numC
                        spaces
                        children <- joints
                        spaces
                        char '}'
                        spaces
                        return (Tree nullJoint{
                                name = name,
                                offset = offset,
                                channels = channels
                        } children)
                endSite = do
                        string endSiteName
                        spaces
                        char '{'
                        spaces
                        string "OFFSET"
                        spaces
                        offset <- vec3
                        spaces
                        char '}'
                        return (Tree nullJoint{
                                name = endSiteName,
                                offset = offset
                        } [])
                        where endSiteName = "End Site"
                channels 0 = return []
                channels numC = do
                        spaces
                        c <- wordP
                        cs <- channels (numC-1)
                        return ((toChan c) : cs) where
                                toChan "Xposition" = Xpos
                                toChan "Yposition" = Ypos
                                toChan "Zposition" = Zpos
                                toChan "Xrotation" = Xrot
                                toChan "Yrotation" = Yrot
                                toChan "Zrotation" = Zrot

newline' = try (string "\r\n") <|> (string "\n")

motion skel = do
        spaces
        string "MOTION"
        spaces
        string "Frames:"
        spaces
        numF <- intP
        spaces
        string "Frame Time:"
        spaces
        frameTime <- floatP
        spaces
        frames <- getFrames
        spaces
        return (frames, frameTime) where
                getFrames = endBy line newline' <?> "Frames"
                line = do
                        vs <- sepBy floatP (many1 (char ' '))
                        return (fromChanVals skel vs)
                      <?> "Line"
        

bvh = do
        spaces
        skel <- fmap toFocus hierarchy
        (frames,frameTime) <- motion skel
        return MotionData {
                frames = frames,
                baseSkeleton = skel,
                frameTime = frameTime
        }

raw_parseBVH :: String -> IO(MotionData)
raw_parseBVH filePath = do
        content <- readFile filePath
        out <- either (error $ "Failed to parse bvh file: " ++ filePath) (return) (parse bvh "" content)
        print "done Parsing"
        return out