module FFI (
    c_main,
    initOgre,
    drawBone,
    drawBoneC,
    drawVec3,
    drawVec3C,
    drawPoint,
    drawPointC,
    doRender
) where


import Foreign.C
import Linear.V3
import Util
import Data.Color


foreign import ccall unsafe "ODE_01.h"
        c_main :: IO()

foreign import ccall unsafe "Interface.h"
        initOgre :: IO ()
        
foreign import ccall unsafe "Interface.h doRender"
        doRender_c :: IO (CInt)
doRender :: IO (Bool)
doRender = fmap ((/=0) . fromIntegral) doRender_c
        
foreign import ccall unsafe "Interface.h drawBone"
        drawBone_c :: CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> IO ()
drawBoneC :: Color -> Vec3 -> Vec3 -> Double -> IO ()
drawBoneC = fromFnCVec3Vec3D drawBone_c
drawBone :: Vec3 -> Vec3 -> Double -> IO ()
drawBone = drawBoneC White
        
foreign import ccall unsafe "Interface.h drawVec3"
        drawVec3_c :: CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> IO ()
drawVec3C :: Color -> Vec3 -> Vec3 -> Double -> IO ()
drawVec3C = fromFnCVec3Vec3D drawVec3_c
drawVec3 :: Vec3 -> Vec3 -> Double -> IO ()
drawVec3 = drawVec3C White
        
foreign import ccall unsafe "Interface.h drawPoint"
        drawPoint_c :: CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> IO ()
drawPointC :: Color -> Vec3 -> Double -> IO ()
drawPointC = fromFnCVec3D drawPoint_c
drawPoint :: Vec3 -> Double -> IO ()
drawPoint = drawPointC White
        
fromFnCVec3Vec3D :: (CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> IO ()) ->  Color -> Vec3 -> Vec3 -> Double -> IO ()
fromFnCVec3Vec3D fn color (V3 x1 y1 z1) (V3 x2 y2 z2) d = let (r, g, b, a) = toRGBA color in
    fn
        (CDouble r)
        (CDouble g)
        (CDouble b)
        (CDouble a)
        (CDouble x1)
        (CDouble y1)
        (CDouble z1)
        (CDouble x2)
        (CDouble y2)
        (CDouble z2)
        (CDouble d)
        
fromFnCVec3D :: (CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> IO ()) -> Color -> Vec3 -> Double -> IO ()
fromFnCVec3D fn color (V3 x y z) d =  let (r, g, b, a) = toRGBA color in
    fn
        (CDouble r)
        (CDouble g)
        (CDouble b)
        (CDouble a)
        (CDouble x)
        (CDouble y)
        (CDouble z)
        (CDouble d)


