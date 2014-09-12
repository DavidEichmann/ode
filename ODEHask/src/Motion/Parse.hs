module Motion.Parse (
    parseBVH
) where

import Util hiding (vec3)
import Motion.MotionData
import Data.TreeF
import Motion.Joint
import Constants


import Data.Array
import Linear
import Data.Char
import Text.Parsec
import Text.Parsec.Token hiding (dot)
import Text.Parsec.Language

{-------------------------

        HIGH LEVEL
        FUNCTIONS

--------------------------}



parseBVH :: String -> Integer -> IO (MotionData)
parseBVH filePath ppFilterWindow = do
        bvh <- raw_parseBVH filePath
        return $ (if ppFilterWindow > 0 then preProcess ppFilterWindow else id) bvh

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
        -- As most motion data has the first frame as a rest pose, we prune that frame
        frames <- fmap tail getFrames
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
                frames = listArray (0,(length frames)-1) frames,
                baseSkeleton = skel,
                frameTime = frameTime
        }

raw_parseBVH :: String -> IO(MotionData)
raw_parseBVH filePath = do
        content <- readFile filePath
        out <- either (error $ "Failed to parse bvh file: " ++ filePath) (return) (parse bvh "" content)
        print "done Parsing"
        return out
