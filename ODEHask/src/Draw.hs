module Draw where

import Linear hiding (trace)
import Util
import Data.Color
import Data.Bone
import Data.TreeF
import FFI
import Motion
import Constants
import Debug.Trace

showOrient = False


drawPolygonEdgesC :: Color -> [Vec3] -> IO ()
drawPolygonEdgesC _ [] = return ()
drawPolygonEdgesC c poly = mapM_ (\(a,b) -> drawBoneC c a b 0.005) (zip poly ((tail poly) ++ [head poly]))

drawFrameIx ::  Color -> Vec3 -> MotionDataVars -> FrameIx -> IO ()
drawFrameIx c offset' MotionDataVars{_bs=bs,_rb=rb,_xb=xb,_xsb=xsb,_xeb=xeb} fi = mapM_ drawBone' bs where
     drawBone' :: BoneIx -> IO ()
     drawBone' bi = do
        drawBoneC c (offset' + xsb fi bi) (offset' + xeb fi bi) boneRadius
        if showOrient
            then do
                drawBoneC c (offset' + xb fi bi) (offset' + ((xb fi bi) + ((rb fi bi) `rotate` (V3 0.1 0 0)))) (boneRadius/3)
                drawBoneC c (offset' + xb fi bi) (offset' + ((xb fi bi) + ((rb fi bi) `rotate` (V3 0 0.1 0)))) (boneRadius/3)
            else
                return ()


drawFrame ::  Vec3 -> Frame -> IO ()
drawFrame offset' j = treeMapM_ drawBone' (set ((view j){offset = (offset $- j) + offset'}) j) where
     drawBone' :: JointF -> IO ()
     drawBone' j
            | hasParent j   = do
                -- Draw bone
                drawBoneC (BlueA 0.5) (getPosStart j) (getPosEnd j) boneRadius
                -- Draw an orientation visualization
                let
                    orientStart = getPosCom j
                    orientDirection = orientStart + (0.1 *^ normalize $ ((getPosEnd j) - (getPosStart j)) `cross` (V3 0.31524 0.1039 1))
                drawBoneC (BlueA 0.5) orientStart orientDirection (boneRadius/3)
            | otherwise     = return ()

drawSkeleton :: Color -> [Bone] -> IO ()
drawSkeleton color = mapM_ drawBone' where
    drawBone' :: Bone -> IO ()
    drawBone' (Long start end) = drawBoneC color start end boneRadius
    drawBone' (Box lengths center rot) = drawBoxC color lengths center rot
