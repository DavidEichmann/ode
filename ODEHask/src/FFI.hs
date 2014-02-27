module FFI (
    c_main,
    initOgre,
    drawBone,
    drawVec3,
    drawPoint,
    doRender
) where


import Foreign.C
import Linear.V3
import Util


foreign import ccall unsafe "ODE_01.h"
        c_main :: IO()

foreign import ccall unsafe "ODE_01.h"
        initOgre :: IO ()
        
foreign import ccall unsafe "ODE_01.h doRender"
        doRender_c :: IO (CInt)
doRender :: IO (Bool)
doRender = fmap ((/=0) . fromIntegral) doRender_c
        
foreign import ccall unsafe "ODE_01.h drawBone"
        drawBone_c :: CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> IO ()
drawBone :: Vec3 -> Vec3 -> Double -> IO ()
drawBone = fromFnVec3Vec3D drawBone_c
        
foreign import ccall unsafe "ODE_01.h drawVec3"
        drawVec3_c :: CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> IO ()
drawVec3 :: Vec3 -> Vec3 -> Double -> IO ()
drawVec3 = fromFnVec3Vec3D drawVec3_c
        
foreign import ccall unsafe "ODE_01.h drawPoint"
        drawPoint_c :: CDouble -> CDouble -> CDouble -> CDouble -> IO ()
drawPoint :: Vec3 -> Double -> IO ()
drawPoint = fromFnVec3D drawPoint_c
        
fromFnVec3Vec3D :: (CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> IO ()) -> Vec3 -> Vec3 -> Double -> IO ()
fromFnVec3Vec3D fn (V3 x1 y1 z1) (V3 x2 y2 z2) d = 
    fn
        (CDouble x1)
        (CDouble y1)
        (CDouble z1)
        (CDouble x2)
        (CDouble y2)
        (CDouble z2)
        (CDouble d)
        
fromFnVec3D :: (CDouble -> CDouble -> CDouble -> CDouble -> IO ()) -> Vec3 -> Double -> IO ()
fromFnVec3D fn (V3 x y z) d = 
    fn
        (CDouble x)
        (CDouble y)
        (CDouble z)
        (CDouble d)


