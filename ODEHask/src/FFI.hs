module FFI (
    c_main,

    inverseMatrix,
    sparseMatrixSolve,
    leastSquareSparseMatrixSolve,

    initOgre,
    drawPolygonC,
    drawBone,
    drawBoneC,
    drawBox,
    drawBoxC,
    drawVec3,
    drawVec3C,
    drawPoint,
    drawPointC,
    doRender,


    DWorldID,
    DBodyID,
    DJointID,
    initODE,
    getCoP,
    getFloorContacts,
    getBodyPosRot,
    setBodyPosRot,
    getBodyBone,
    appendCapsuleBody,
    appendFootBody,
    createBallJoint,
    createAMotor,
    setAMotorVelocity,
    addAMotorTorque,
    createFixedJoint,
    stepODE
) where


import Foreign hiding (unsafeLocalState)
import Foreign.C
import Foreign.Marshal.Unsafe
import Linear
import Util
import Data.Color
import Data.Maybe
import Data.Bone
import Data.Array
import Data.List.Split
import Control.Arrow
import Control.Monad

foreign import ccall unsafe "ODE_01.h"
        c_main :: IO()

foreign import ccall unsafe "Interface.h"
        initOgre :: IO ()

foreign import ccall unsafe "Interface.h doRender"
        doRender_c :: IO (CInt)
doRender :: IO (Bool)
doRender = fmap ((/=0) . fromIntegral) doRender_c


foreign import ccall unsafe "Interface.h drawPolygon"
    drawPolygon_c :: CDouble -> CDouble -> CDouble -> CDouble -> CInt -> Ptr CDouble -> IO ()
drawPolygonC :: Color -> [Vec3] -> IO ()
drawPolygonC c poly = do
    let n = length poly
    allocaArray (3*n) (\arrVals -> do
        -- fill array with Vec3 values
        pokeArray arrVals (map CDouble $ concat $ map (\(V3 x y z) -> [x,y,z]) poly)
        -- draw
        ((applyColor c) >>> (apply (fromIntegral n)) >>> (apply arrVals)) drawPolygon_c
     )
drawPolygon :: [Vec3] -> IO ()
drawPolygon = drawPolygonC White

foreign import ccall unsafe "Interface.h drawBox"
    drawBox_c :: CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> IO ()
drawBoxC :: Color -> Vec3 -> Vec3 -> Quat -> IO ()
drawBoxC c size center rot = ((applyColor c) >>> (applyVec3 size) >>> (applyVec3 center) >>> (applyQuat rot)) drawBox_c
drawBox :: Vec3 -> Vec3 -> Quat -> IO ()
drawBox = drawBoxC White

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


{-
    ODE stuff
-}
type CBool = CInt
toCBool b = CInt (if b then 1 else 0)
type DBodyID = Ptr ()
type DJointID = Ptr ()
type DWorldID = Ptr ()
cdPeekArray n = (fmap (map realToFrac)) . (peekArray n)
foreign import ccall unsafe "Interface.h initODE"
        initODE_c :: CDouble -> IO DWorldID
initODE :: Double -> IO DWorldID
initODE timeStep = (applyDouble timeStep) initODE_c

foreign import ccall unsafe "Interface.h getBodyPosRot"
        getBodyPosRot_c :: DBodyID -> IO (Ptr CDouble)
getBodyPosRot :: DBodyID -> IO (Vec3, Quat)
getBodyPosRot bid = do
    (x:y:z:w:qx:qy:qz:_) <- getBodyPosRot_c bid >>= cdPeekArray 7
    return (V3 x y z, Quaternion w (V3 qx qy qz))

foreign import ccall unsafe "Interface.h setBodyPosRot"
        setBodyPosRot_c :: DBodyID -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> IO ()
setBodyPosRot :: DBodyID -> Vec3 -> Quat -> IO ()
setBodyPosRot bid pos rot = ((apply bid) >>> (applyVec3 pos) >>> (applyQuat rot)) setBodyPosRot_c

foreign import ccall unsafe "Interface.h getCoP"
    getCoP_c :: IO (Ptr CDouble)
getCoP :: IO (Vec3)
getCoP = do
    [x,y,z] <- getCoP_c >>= cdPeekArray 3
    return $ V3 x y z

foreign import ccall unsafe "Interface.h getBodyGeom"
        getBodyGeom_c :: DBodyID -> IO (Ptr CDouble)
getBodyBone :: DBodyID -> IO Bone
getBodyBone bid = do
    (cls:x1:y1:z1:x2:y2:z2:qw:qx:qy:qz:_) <- getBodyGeom_c bid >>= cdPeekArray 12
    case round cls of
        1   -> return (Box (V3 x1 y1 z1) (V3 x2 y2 z2) (Quaternion qw (V3 qx qy qz)))
        2   -> return (Long (V3 x1 y1 z1) (V3 x2 y2 z2))

foreign import ccall unsafe "Interface.h getFloorContacts"
        getFloorContacts_c :: IO (Ptr CDouble)
getFloorContacts :: IO [Vec2]
getFloorContacts = do
    ncs <- getFloorContacts_c
    (nd:[]) <- cdPeekArray 1 ncs
    let n = round nd
    (_:cs2) <- cdPeekArray ((n*2)+1) ncs
    return $ map (\(x:z:_) -> V2 x z) (chunksOf 2 cs2)




foreign import ccall unsafe "Interface.h appendCapsuleBody"
        appendCapsuleBody_c :: CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble
                                -> IO DBodyID
appendCapsuleBody :: Vec3 -> Quat -> Quat -> Double -> Double -> Double -> M3 -> IO DBodyID
appendCapsuleBody
    -- position CoM
    com
    -- local rotation for the bone from Z aligned
    lRotZ
    -- global quaternion rotation
    gRot
    -- dimensions, mass
    radius length mass
    -- Inertia matrix (about CoM)
    (V3     (V3 i11 i12 i13)
            (V3  _  i22 i23)
            (V3  _   _  i33))

        =  apps appendCapsuleBody_c where
                apps =  (applyVec3 com) >>>
                        (applyQuat2 lRotZ gRot) >>>
                        (applyDouble9 radius length mass i11 i12 i13 i22 i23 i33)

foreign import ccall unsafe "Interface.h appendFootBody"
    appendFootBody_c :: CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble
                            -> IO DBodyID
appendFootBody :: Vec3 -> Quat -> Vec3 -> Double -> M3 -> IO DBodyID
appendFootBody
     -- position CoM
    com
    -- global quaternion rotation
    rot
    -- dimensions
    boxSize
    -- mass and Inertia matrix (about CoM)
    mass
    (V3     (V3 i11 i12 i13)
            (V3  _  i22 i23)
            (V3  _   _  i33))
        = ((applyVec3 com) >>>
            (applyQuat rot) >>>
            (applyVec3 boxSize) >>>
            (applyDouble7 mass i11 i12 i13 i22 i23 i33)) appendFootBody_c

foreign import ccall unsafe "Interface.h createBallJoint"
    createBallJoint_c :: DBodyID -> DBodyID -> CDouble -> CDouble -> CDouble -> IO ()
createBallJoint :: DBodyID -> DBodyID -> Vec3 -> IO ()
createBallJoint a b pos = applyVec3 pos (createBallJoint_c a b)

foreign import ccall unsafe "Interface.h createAMotor"
    createAMotor :: DBodyID -> DBodyID -> IO (DJointID)

foreign import ccall unsafe "Interface.h setAMotorVelocity"
    setAMotorVelocity_c :: DJointID -> CDouble -> CDouble -> CDouble -> IO ()
setAMotorVelocity :: DJointID -> Vec3 -> IO ()
setAMotorVelocity jid aVel = ((apply jid) >>> (applyVec3 aVel)) setAMotorVelocity_c

foreign import ccall unsafe "Interface.h addAMotorTorque"
    addAMotorTorque_c :: DJointID -> CDouble -> CDouble -> CDouble -> IO ()
addAMotorTorque :: DJointID -> Vec3 -> IO ()
addAMotorTorque joint torque = ((apply joint) >>> (applyVec3 torque)) addAMotorTorque_c

foreign import ccall unsafe "Interface.h createFixedJoint"
    createFixedJoint :: DBodyID -> DBodyID -> IO ()

apply a f = f a
applyBool b = apply (CInt (if b then 1 else 0))
applyDouble v = apply (CDouble v)
applyDouble2 a b = (applyDouble a) >>> (applyDouble b)
applyDouble3 a b c = (applyDouble2 a b) >>> (applyDouble c)
applyDouble4 a b c d = (applyDouble3 a b c) >>> (applyDouble d)
applyDouble5 a b c d e = (applyDouble4 a b c d) >>> (applyDouble e)
applyDouble6 a b c d e f = (applyDouble5 a b c d e) >>> (applyDouble f)
applyDouble7 a b c d e f g = (applyDouble6 a b c d e f) >>> (applyDouble g)
applyDouble8 a b c d e f g h = (applyDouble7 a b c d e f g) >>> (applyDouble h)
applyDouble9 a b c d e f g h i = (applyDouble8 a b c d e f g h) >>> (applyDouble i)
applyVec2 (V2 a b) = applyDouble2 a b
applyVec3 (V3 x y z) = applyDouble3 x y z
applyQuat (Quaternion w vec) = (applyDouble w) >>> (applyVec3 vec)
applyQuat2 a b = (applyQuat a) >>> (applyQuat b)
applyColor c = let (r, g, b, a) = toRGBA c in applyDouble4 r g b a

foreign import ccall unsafe "Interface.h step"
    stepODE_c :: DWorldID -> CDouble -> CDouble -> CDouble -> IO ()
-- stepODE (ode world ID) (target ZMP xz coordinates) (net y component of ground reaction forces)
stepODE :: DWorldID -> Vec2 -> Double -> IO ()
stepODE wid zmp fy = ((apply wid) >>> (applyVec2 zmp) >>> (applyDouble fy)) stepODE_c

--foreign import ccall unsafe "Interface.h stressTest"
-- stressTest :: IO ()
--foreign import ccall unsafe "Interface.h stressTestUnit"
-- stressTestUnit_c :: CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> IO ()
--
--stressTestUnit :: Double -> Double -> Double -> Double -> Double -> Double -> Double -> IO ()
--stressTestUnit a b c d e f g = stressTestUnit_c (CDouble a) (CDouble b) (CDouble c) (CDouble d) (CDouble e) (CDouble f) (CDouble g)



fromFnCVec3Vec3D :: (CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> IO ()) ->  Color -> Vec3 -> Vec3 -> Double -> IO ()
fromFnCVec3Vec3D fn a b c d = ((applyColor a) >>> (applyVec3 b) >>> (applyVec3 c) >>> (applyDouble d)) fn
{-fromFnCVec3Vec3D fn color (V3 x1 y1 z1) (V3 x2 y2 z2) d = let (r, g, b, a) = toRGBA color in
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
        (CDouble d)-}

fromFnCVec3D :: (CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> CDouble -> IO ()) -> Color -> Vec3 -> Double -> IO ()
fromFnCVec3D fn a b c = ((applyColor a) >>> (applyVec3 b) >>> (applyDouble c)) fn
{-fromFnCVec3D fn color (V3 x y z) d =  let (r, g, b, a) = toRGBA color in
    fn
        (CDouble r)
        (CDouble g)
        (CDouble b)
        (CDouble a)
        (CDouble x)
        (CDouble y)
        (CDouble z)
        (CDouble d)-}


foreign import ccall unsafe "Interface.h sparseMatrixSolve"
    sparseMatrixSolve_c :: CInt -> CInt -> CInt -> Ptr CInt -> CInt -> Ptr CDouble -> IO (Ptr CDouble)
sparseMatrixSolveRaw :: Int -> Int -> [((Int,Int),Double)] -> [Array Int Double] -> [Array Int Double]
sparseMatrixSolveRaw mrn mcn matrix rhs = unsafeLocalState (do
    let
        -- n = arraySize $ head rhs   -- size of rhs vectors (also matrix is n x n)
        r = length rhs          -- number of rhs vectors

        mNZ = length matrix         -- number of nonempty (non-zero) matix elements
        arrMIxsSize = 2 * mNZ       -- size of matrix index array
        arrSize = (mNZ) + (mrn * r)   -- size of dataArray

        breakSized :: Int -> [a] -> [[a]]
        breakSized s [] = []
        breakSized s a = let (b, rem) = splitAt s a in b : (breakSized s rem)

    allocaArray arrMIxsSize (\arrMIxs -> do
        allocaArray arrSize (\arr -> do
            -- fill matrix index array (in the form: [row1,col1,row2,col2 ... , row mNZ, col mNZ])
            pokeArray arrMIxs (map fromIntegral (concat $ map (\((r,c),_) -> [r,c]) matrix))
            -- fill array with matrix values then rhs vectors
            pokeArray arr (map CDouble ((map snd matrix) ++ (concat $ map elems rhs)))
            results <- cdPeekArray (r*mcn) =<< sparseMatrixSolve_c (fromIntegral mrn) (fromIntegral mcn) (fromIntegral mNZ) arrMIxs (fromIntegral r) arr
            return $ map (listArray (0,mcn-1)) (breakSized mcn results)
         )
     )
 )

sparseMatrixSolve :: Int -> Int -> [((Int,Int),Double)] -> [Array Int Double] -> [Array Int Double]
sparseMatrixSolve = error "only least squares is supported now"
leastSquareSparseMatrixSolve :: Int -> Int -> [((Int,Int),Double)] -> [Array Int Double] -> [Array Int Double]
leastSquareSparseMatrixSolve = sparseMatrixSolveRaw


foreign import ccall unsafe "Interface.h inverseMatrix"
    inverseMatrix_c :: CInt -> CInt -> Ptr CDouble -> IO (Ptr CDouble)
inverseMatrix :: Array (Int,Int) Double -> Array (Int,Int) Double
inverseMatrix m = unsafeLocalState (do
        let
            ((ri,ci),(rf,cf)) = bounds m
            r = 1 + (rf - ri)
            c = 1 + (cf - ci)
        allocaArray (r*c) (\arr -> do
            pokeArray arr (map CDouble $ elems m)
            inv <- cdPeekArray (c*r)  =<< inverseMatrix_c (fromIntegral r) (fromIntegral c) arr
            return (listArray ((ci,ri),(cf,rf)) inv)
         )
    )




