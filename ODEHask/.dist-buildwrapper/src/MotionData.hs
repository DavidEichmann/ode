module MotionData where

import Control.Monad
import Control.Applicative (liftA2)
import Data.List
import Data.Char
import Data.Maybe
import Data.TreeF
import Text.Parsec
import Text.Parsec.Token
import Text.Parsec.Language
import Linear
import Util
import Constants

data Channel = Xpos | Ypos | Zpos | Xrot | Yrot | Zrot deriving (Eq, Show)
posChans :: [Channel]
posChans = [Xpos,Ypos,Zpos]
rotChans :: [Channel]
rotChans = [Xrot,Yrot,Zrot]

--type JointF = TreeF Joint

data Joint = Joint {
                name     :: String,
                offset   :: Vec3,
                rotation :: Quat,
                channels :: [Channel]
        } deriving Show
                
-- this data (mass/inertiaM) refers to the bone between this and parent joint

-- mass is bone length * 10 kg  (root has no mass)
mass :: JointF -> Double
mass jF
        | hasParent jF   = 10 * (norm $ offset $-  jF)
        | otherwise     = 0
        
-- inertia matrix is computed as that of a capsule represented by the offset vector
-- as the root joint does not represent a bone, it has no valid inertia matrix, so zero is used  
inertiaM :: JointF -> M3
inertiaM jF
        | hasParent jF   = zero
        | otherwise     = zero
        
nullJoint :: Joint
nullJoint = Joint {
        name = "",
        offset = zero,
        rotation = identity,
        channels = []
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
fromChanVals jf cv = treeMapCon_ applyChans cv jf where
         applyChans cv jf = (cv', set (sk{ offset = offset', rotation = rotation'}) jf) where
                sk = view jf 
                baseOffset = offset sk
                baseRot    = rotation sk
                (offset', rotation', cv') = consume baseOffset baseRot (channels sk) cv
                consume ost rot [] vs = (ost, rot, vs)
                consume _ _ _ [] = error("Not enough values to fill channels")
                consume (V3 _ py pz) rot (Xpos:cs) (v:vs) = consume (V3 v py pz) rot cs vs
                consume (V3 px _ pz) rot (Ypos:cs) (v:vs) = consume (V3 px v pz) rot cs vs
                consume (V3 px py _) rot (Zpos:cs) (v:vs) = consume (V3 px py v) rot cs vs
                consume pos rot (Xrot:cs) (v:vs) = consume pos (rot * (axisAngle (unitX) (degreeToRadian v))) cs vs
                consume pos rot (Yrot:cs) (v:vs) = consume pos (rot * (axisAngle (unitY) (degreeToRadian v))) cs vs
                consume pos rot (Zrot:cs) (v:vs) = consume pos (rot * (axisAngle (unitZ) (degreeToRadian v))) cs vs
                         

parseBVH :: String -> IO (MotionData)
parseBVH filePath = do
        bvh <- raw_parseBVH filePath
        return bvh



{-------------------------

        HIGH LEVEL
        FUNCTIONS

--------------------------}

getRot :: JointF -> Quat
getRot j = case parent j of
        Nothing  ->  rotation $- j
        Just p   ->  (rotation $- p) * (rotation $- j)
        
getPosEnd :: JointF -> Vec3
getPosEnd j = getPosEnd' j (parent j) where
        getPosEnd' j Nothing = offset $- j
        getPosEnd' j (Just p) = (getPosEnd p) + (getRot p `rotate` (offset $- j))

getPosStart :: JointF -> Vec3
getPosStart j = maybe zero getPosEnd (parent j)

getPosCom :: JointF -> Vec3
getPosCom jf = ((getPosStart jf) + (getPosEnd jf)) / 2 where


-- get the (min, max) coordinates of the system
frameBoundingBox :: Frame -> (Vec3, Vec3)
frameBoundingBox f = treeFold minMax initMinMax f where
        initMinMax = let o = offset $- f in (o, o)
        minMax :: (Vec3, Vec3) -> JointF -> (Vec3, Vec3)
        minMax (cMin,cMax) v = let vPos = getPosEnd v in ((liftA2 min) cMin vPos, (liftA2 max) cMax vPos)



scaleAndTranslate :: Double -> Int -> MotionData -> MotionData
scaleAndTranslate targetHeight sampleFrameIndex md = mdTS where
    bb = frameBoundingBox $ (frames md)!!sampleFrameIndex
    bbDiag = uncurry (+) bb
    bbC = bbDiag / 2
    targetC = scaleFactor *^ (V3 0 (2 + boneRadius) 0)
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
                        return (Tree Joint{
                                name = name,
                                offset = offset,
                                rotation = identity,
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
                        return (Tree Joint{
                                name = endSiteName,
                                offset = offset,
                                rotation = identity,
                                channels = []
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
                getFrames = endBy line newline <?> "Frames"
                line = do
                        vs <- sepBy floatP (many1 (char ' '))
                        return (fromChanVals skel vs)
                      <?> "Line"
                vals = many (floatP)
        

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
        out <- either (\e -> do print e; return (MotionData{})) (return) (parse bvh "" content)
        print "done Parsing"
        return out