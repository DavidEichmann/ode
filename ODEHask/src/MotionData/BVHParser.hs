{-module MotionData.BVHParser(
    raw_parseBVH
) where

import Data.List
import Data.Char
import Text.Parsec
import Text.Parsec.Token
import Text.Parsec.Language
import Linear
import Util
import MotionData (Joint, JointTree, MotionData)


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
                        return (JointTree Joint{
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
                        return (JointTree Joint{
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
 -}
 
