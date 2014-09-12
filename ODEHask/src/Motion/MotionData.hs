module Motion.MotionData where

import Control.Applicative (liftA2)
import Data.Array
import Data.TreeF
import Linear

import Util
import Motion.Util
import Motion.Joint
import Constants


type JointF = (TreeF Joint)
type Frame = JointF

data MotionData = MotionData {
        frames        :: Array Int Frame,
        frameTime     :: Double,
        baseSkeleton  :: JointF
} deriving Show





jointScale :: Double -> JointF -> JointF
jointScale s = treeMap (map1 (\j@Joint{offset=o} -> j{offset=o ^* s}))

jointTranslate :: Vec3 -> JointF -> JointF
jointTranslate t = map1 (\r@Joint{offset=o} -> r{offset=o+t})

scale :: Double -> MotionData -> MotionData
scale s md = md{
                frames = fmap (jointScale s) (frames md),
                baseSkeleton = jointScale s (baseSkeleton md)
        }

translate :: Vec3 -> MotionData -> MotionData
translate t md = md{
                frames = fmap (jointTranslate t) (frames md),
                baseSkeleton = jointTranslate t (baseSkeleton md)
        }


-- Legacy code
-- inertia matrix is computed as that of a capsule represented by the offset vector
-- as the root joint does not represent a bone, it has no valid inertia matrix, so zero is used
massInertiaM :: JointF -> (Double, M3)
massInertiaM jF
        | hasParent jF   = getCapsulMassInertiaM (density jF) boneRadius (offset $- jF)
        | otherwise     = (0,zero)


-- legacy code
density :: JointF -> Double
density jF
        | hasParent jF   = boneDensity
        | otherwise     = 0
        
        
        
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

scaleAndTranslate :: MotionData -> MotionData
scaleAndTranslate md = mdTS where
    bb = frameBoundingBox $ (frames md)!0
    bbDiag = uncurry (+) bb
    bbC@(V3 _ cHeight _) = bbDiag / 2
    targetC = (V3 0 (cHeight + boneRadius) 0)
    translation = targetC - bbC
    scaleFactor = capturedDataScale

    mdT  = translate translation md
    mdTS = scale scaleFactor mdT


scaleAndTranslateToDesiredHeight :: Double -> Int -> MotionData -> MotionData
scaleAndTranslateToDesiredHeight targetHeight sampleFrameIndex md = mdTS where
    bb = frameBoundingBox $ (frames md)!sampleFrameIndex
    bbDiag = uncurry (+) bb
    bbC@(V3 _ cHeight _) = bbDiag / 2
    targetC = (V3 0 (cHeight + boneRadius) 0)
    translation = targetC - bbC
    scaleFactor = let V3 _ height _ = bbDiag in targetHeight / height

    mdT  = translate translation md
    mdTS = scale scaleFactor mdT


getInterpolatedFrame :: Double -> MotionData -> Frame
getInterpolatedFrame t md
    | i > snd (bounds (frames md))  = error "getInterpolatedFrame: i out of bounds"
    | j > snd (bounds (frames md))  = error "getInterpolatedFrame: j out of bounds"
    | otherwise = f where
        n = arraySize $ frames md
        i' = doMod (t / (frameTime md)) + 1 where
            nd = fromIntegral $ n
            doMod x
                | x < nd = x
                | otherwise = doMod (x - nd)
        i = min (n-1) (floor i')
        j = min (n-1) (i+1)
        a = (frames md) ! i
        b = (frames md) ! j
        f = set ((view f'){offset = ((offset $- a) + (offset $- b)) ^/ 2}) f'
        f' = treeZipWith interpJoint a b
        interp = i' - (fromIntegral i)
        interpJoint af bf = (view af){
                                rotationL  = Util.slerp (rotationL $- af) (rotationL $- bf) interp
                            }


isFootJoint :: JointF -> Bool
isFootJoint jf = (name $- jf) `elem` footJoints || (name $- (justParent jf)) `elem` footJoints where
    footJoints = ["LeftFoot", "LeftToe", "RightFoot", "RightToe"]
isToeJoint :: JointF -> Bool
isToeJoint jf = hasParent jf && (name $- (justParent jf)) `elem` footJoints where
    footJoints = ["LeftToe", "RightToe"]
        
        
-- currently this just uses moving average for joint angles and positions
preProcess :: Integer -> MotionData -> MotionData
preProcess windowSize md
    | newBounds /= bounds (frames md)   = error "preprocessing: new bounds not equal to old bounds!!!"
    | otherwise = mdF where
        n = fromInteger windowSize
        newFrames = map avg (windows (elems $ frames md))
        newBounds = (0, (length newFrames) - 1)
        mdF = md{
                 frames = listArray newBounds newFrames
            }
        windows fs = lfsn : if length lfsn == 1 then [] else windows (tail fs)
            where
                lfsn = take n fs
        avg fs = fmap avgJ (foldl1 (treeZipWith (\a b -> (view a) ++ (view b))) (map (fmap (:[])) fs))
        avgJ :: [Joint] -> Joint
        avgJ js = (head js) {
            offset   = avgPos,
            rotationL = avgRotL
        }  where
            avgPos = (foldl1 (+) (map offset js)) ^/ (fromIntegral (length js))
            avgRotL = avgRot (map rotationL js)