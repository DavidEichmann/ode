module Test.MyTest where

import Test.HUnit
import Data.Vector (fromList)
import qualified Data.Vector as V

import Util
import Motion
import Constants
import Linear hiding (_w,_l)
import Test.TestData
import Test.HUnitX


-- load MotionDataVars from data file with scaling and translation, but no preprocessing
loadScaledData :: String -> IO MotionDataVars
loadScaledData file = fmap (getMotionDataVariablesFromMotionData . scaleAndTranslate) $ parseBVH file 0

-- load MotionDataVars from data file WITHOUT scaling and translation, and no preprocessing
-- This is important to preserve e.g. the acceleration of gravity
loadRawData :: String -> IO MotionDataVars
loadRawData file = fmap getMotionDataVariablesFromMotionData $ parseBVH file 0

runTests :: IO ()
runTests = do
    
    -- static
    staticBodyMDV           <- loadScaledData (defaultTestDataDirectory ++ "static_body.bvh")
    -- constant linear velocity static-pose
    constVelocityBodyMDV    <- loadScaledData (defaultTestDataDirectory ++ "constant_velocity_body.bvh")
    -- constant angular velocity static-pose
    constWRootBodyMDV       <- loadScaledData (defaultTestDataDirectory ++ "constant_angular_root_velocity_body.bvh")
    
    -- g = 10 free falling body constant-pose
    g10DropBodyMDV          <- loadScaledData (defaultTestDataDirectory ++ "g10_drop_body.bvh")
    
    -- static 1 link horizontal chain
    static1LinkChainMDV     <- loadRawData    (defaultTestDataDirectory ++ "static_1_link_chain.bvh")
    -- g = 10 free falling 1 link chain
    g10Drol1LinkChainMDV    <- loadRawData    (defaultTestDataDirectory ++ "g10_drop_1_link_chain.bvh")
    -- static 2 link horizontal chain
    static2LinkChainMDV     <- loadRawData    (defaultTestDataDirectory ++ "static_2_link_chain.bvh")
    
    -- Run all the tests!!!
    Counts{errors=errors,failures=failures} <- runTestTT $ TestList [
        
            "Utilities" ~: test [
                "zToDirQuat" ~: test [((q `rotate` (unitZ ^* (norm dr))) @?~=: dr) ("quaternion: " ++ show q ++ "\nvector: " ++ show dr) | dr <- testDataVec3, let q = zToDirQuat dr]
                ,"dirToDirQuat" ~: sequence_ [((dirToDirQuat b a) `rotate` ((norm a) *^ (normalize b)) @?~=: a) ("a: " ++ show a ++ "\nb: " ++ show b) | a <- testDataVec3, a /= 0, b <- testDataVec3, b /= 0]
            ]
        
            -- Test the MotionDataVars
            
            ,"MotionDataVars" ~: test [
                "Static Body" ~: let smdv@MotionDataVars{_fs=fs,_bs=bs,_js=js,_l=l,_w=w,_q'=q',_q''=q''} = staticBodyMDV in test [
                
                             "For all Frames fI and Bones bI, l fI bI == V3 0 0 0" ~:
                                sequence_ [l fI bI @?= V3 0 0 0 | fI <- fs, bI <- bs]
                            ,"For all Frames fI and Bones bI, w fI bI == V3 0 0 0" ~:
                                sequence_ [w fI bI @?= V3 0 0 0 | fI <- fs, bI <- bs]
                            ,"For all Frames fI and Joints jI, q' fI bI == (V3 0 0 0,V3 0 0 0)" ~:
                                sequence_ [q' fI jI @?= (vec3 0 0 0, vec3 0 0 0) | fI <- fs, jI <- js]
                            ,"For all Frames fI and Joint jI, q'' fI bI == (V3 0 0 0,V3 0 0 0)" ~:
                                sequence_ [q'' fI jI @?= (vec3 0 0 0, vec3 0 0 0) | fI <- fs, jI <- js]
                ]
                                
                ,"Constant Velocity Body" ~: let smdv@MotionDataVars{_fs=fs,_bs=bs,_js=js,_l=l,_w=w,_q'=q',_q''=q''} = constVelocityBodyMDV in test [
                
                             "For all Frames fI and Bones bI, l (F 0) bI == l fI bI" ~:
                                sequence_ [l (F 0) bI @?~= l fI bI | fI <- fs, bI <- bs]
                            ,"For all Frames fI and Bones bI, w fI bI == V3 0 0 0" ~:
                                sequence_ [w fI bI @?~= V3 0 0 0 | fI <- fs, bI <- bs]
                            ,"For all Frames fI and Joints jI other than the root, q' fI jI == (V3 0 0 0,V3 0 0 0)" ~:
                                sequence_ [q' fI jI @?~= (vec3 0 0 0, vec3 0 0 0) | fI <- fs, jI <- tail js]
                            ,"For all Frames fI, q' (F 0) (J 0) == q' fI (J 0)" ~:
                                sequence_ [q' fI (J 0) @?~= q' (F 0) (J 0) | fI <- fs, jI <- tail js]
                            ,"For all Frames fI, angular q' (F 0) (J 0) != zero" ~:
                                sequence_ [fst (q' fI (J 0)) @?~= vec3 0 0 0 | fI <- fs]
                            ,"For all Frames fI, linear q' (F 0) (J 0) == zero" ~:
                                sequence_ [snd (q' fI (J 0)) @?!~= (vec3 0 0 0) | fI <- fs]
                            ,"For all Frames fI and Joint jI, q'' fI bI == (V3 0 0 0,V3 0 0 0)" ~:
                                sequence_ [q'' fI jI @?~= (vec3 0 0 0, vec3 0 0 0) | fI <- fs, jI <- js]
                                
                ]
                
                ,"Spinning Root Body" ~: let smdv@MotionDataVars{_fs=fs,_bs=bs,_js=js,_l=l,_w=w,_q'=q',_q''=q''} = constWRootBodyMDV in test [
                
                            "For all Frames fI and Bones bI, w (F 0) (B 1) == w fI bI" ~:
                                sequence_ [w (F 0) (B 1) @~=? w fI bI | fI <- fs, bI <- tail bs]
                                
                                
                                {- it is not clear if q' and q'' are meant to be local or global (Note that w is correct but is defined in terms of q', make sure to keep w/w' when fixing q'/q'')
                            ,"For all Frames fI and Joints jI, q' fI bI == V3 0 0 0" ~:
                                test [("(fI,jI) = (" ++ show fI ++ "," ++ show jI ++ ")") ~: q' fI jI @?= V3 0 0 0 | fI <- fs, jI <- js]
                            ,"For all Frames fI and Joint jI, q'' fI bI == V3 0 0 0" ~:
                                sequence_ [q'' fI jI @?= V3 0 0 0 | fI <- fs, jI <- js]
                                -}
                
                ]
            ]
            
            ,"InverseDynamics" ~: test [
            
                            unitTestsID
                            
                            ,"Zero Gravity, Static Body (Frame 5)" ~: let 
                              MotionDataVars{_js=js,_bs=bs,_xj=xj,_xsb=xsb} = staticBodyMDV
                              fI = F 5
                              ((jointTorques,_),(axb,(xfOj,(ixpi,(inert,(ixpiF,(v,(a,_)))))))) = inverseDynamicsInternals False 0 staticBodyMDV fI
                               in test [
                            
                                "Basis conversion" ~: test [
                                
                                    "Changing to the same basis results in the same spatial motion (axb)" ~: let x = axb (fromList [0, 0, 0]) identity in
                                        sequence_ [ (x !* p) @?~= p | p <- testDataVector6 ]
                                
                                    ,"Changing to a rotated basis results in a rotated spatial motion (axb)" ~: let rot = axisAngle (V3 1 2 3) 90; rotM = quat2Matrix rot; x = axb (fromList [0, 0, 0]) rot in
                                        sequence_ $ concat [ [w' @?~= (rotM !* w), l' @?~= (rotM !* l)] | p <- testDataVector6, let (w,l) = V.splitAt 3 p, let (w',l') = V.splitAt 3 (x !* p) ]
                                
                                    ,"Changing to a translated basis results in predictable spatial motion (axb)" ~: let trans = fromList [5, 0, 0]; x = axb ((-1) *^ trans) identity in
                                        ((x !* (fromList [1,2,3, 0,0,0])) @?~= (fromList [1,2,3, 0,-15,10]))
                                
                                    ,"spatial force has the same norm of the linear component when changing basis with (xf0j)" ~:
                                        sequence_ [ (norm $ V.drop 3 $ (xfOj jI) !* (fromList [0,0,0,fx,fy,fz])) @?~= normf |
                                                    f@(V3 fx fy fz) <- testDataVec3,
                                                    let normf = norm f,
                                                    jI              <- js ]
                                
                                    ,"spatial movement has the same norm of the angular component when changing basis with (ixpi)" ~:
                                        sequence_ [ (norm $ V.take 3 $ (ixpi bI) !* (fromList [mx,my,mz,0,0,0])) @?~= normm |
                                                    m@(V3 mx my mz) <- testDataVec3,
                                                    let normm = norm m,
                                                    bI              <- bs ]
                                                    
                                    ,"Pure Force should results in spatial force with equal norm linear component, and predictable angular component (when converted to joint basis (pure force <-> 0 couple))" ~:
                                        sequence_ [(do (norm lin) @?~= (norm f); (norm ang) @?~= (norm c);) |
                                                    f@(V3 fx fy fz) <- testDataVec3, 
                                                    jI              <- js,
                                                    let c = cross f (xj fI jI),
                                                    let (ang,lin) = V.splitAt 3 ((xfOj jI) !* (fromList [0,0,0,fx,fy,fz]))]
                                    
                                    ,"In Zero gravity, all torques should be 0" ~: 
                                        sequence_ [(jointTorques jI @?~= (V3 0 0 0)) | jI <- js]
                                    
                                ]
                               
                                ,"Calculated velocities are 0?" ~:
                                        sequence_ [ (v bI) @?~= (fromList [0,0,0,0,0,0]) | bI <- bs]
                                
                            
                            ]
                            
                            ,"Gravity = 10 m/s2, free falling static-pose body" ~: let
                              mdv = g10DropBodyMDV
                              g=10
                              MotionDataVars{_js=js,_fs=fs,_bs=bs,_m=m,_l'=l',_rb=rb,_bj=bj,_pj=pj,_q''=q'',_xsb=xsb,_xeb=xeb} = mdv
                              fI = F 50
                              ((jointTorques,_),(axb,(xfOj,(ixpi,(inert,(ixpiF,(v,(a,(f',(fNet,_)))))))))) = inverseDynamicsInternals False g mdv fI
                               in test [
                               
                                "DATA CHECK! is body really falling with the acceleration of gravity (V3 0 (-10) 0) ?" ~:
                                        sequence_ [ (l' fI bI) @?~= (V3 0 (-g) 0) | bI <- bs]
                               
                                ,"For all Frames fI, linear q'' fI (J 0) == (0 -10 0)" ~:
                                    sequence_ [snd (q'' fI (J 0)) @?!~= (vec3 0 (-g) 0) | fI <- fs]
                                    
                                ,"Calculated spatial accelerations are that of gravity ( 0 (-10) 0)?" ~:
                                        sequence_ [ ((a bI) @?~=: (fromList [0,0,0,x,y,z])) ("bI = " ++ show bI ++ "\nq''=" ++ show (q'' fI (pj $ bj $ bI))) | bI <- bs, let (V3 x y z) = (conjugate (rb fI bI)) `rotate` (V3 0 (-g) 0)]
                               
                               {- this test is wrong!!!! angle of gravity force is not accounted for
                               ,"all fNet = fGravity has g*m norm linear force and g*m*(dist to CoM) norm angular" ~:
                                        sequence_ [((norm a,norm l) @?~=: (gm*d,gm)) ("bI = " ++ show bI) | bI <- bs, let (a,l) = V.splitAt 3 (fNet bI), let gm = g*(m bI), let d = norm ((xsb fI bI) - (xeb fI bI)) /2]
                                -}
                               
                               ,"all f' = net force - external force = net force - force gravity = 0" ~:
                                        sequence_ [((f' bI) @?~=: (fromList [0,0,0,0,0,0])) ("bI = " ++ show bI) | bI <- bs]
                               
                                ,"All joint torques are 0" ~: 
                                        test [(jointTorques jI @?~=: (V3 0 0 0)) ("jI = " ++ show jI) | jI <- js]
                               
                               ]
                            
                            ,"Gravity = 10 m/s2, static 1 link horizontal chain" ~: let
                              mdv = static1LinkChainMDV
                              MotionDataVars{_bN=bN,_js=js,_bs=bs,_l'=l',_m=m,_xb=xb,_xsb=xsb,_q'=q',_q''=q''} = mdv
                              fI = F 4
                              bI = B 0
                              g = 10
                              ((jointTorques,_),(axb,(xfOj,(ixpi,(inert,(ixpiF,(v,(a,(f',(fNet,_)))))))))) = inverseDynamicsInternals False g mdv fI
                               in test [
                               
                                "DATA CHECK! accelerations are 0 ?" ~:
                                        sequence_ [ (l' fI bI) @?~= (vec3 0 0 0) | bI <- bs]
                               
                                ,"DATA CHECK! bN is 1 ?" ~:
                                        bN @?= 1
                               
                                ,"DATA CHECK! q' is 0 ?" ~:
                                        sequence_ [ (q' fI jI) @?~= (vec3 0 0 0, vec3 0 0 0) | jI <- js]
                               
                                ,"DATA CHECK! q'' is 0 ?" ~:
                                        sequence_ [ (q'' fI jI) @?~= (vec3 0 0 0, vec3 0 0 0) | jI <- js]
                                        
                               ,"all bones' net forces fNet = 0" ~:
                                        sequence_ [((fNet bI) @?~=: (fromList [0,0,0,0,0,0])) ("bI = " ++ show bI) | bI <- bs]
                               
                                ,"torque at J 0 (the only joint) is as expected" ~: 
                                        ( (jointTorques (J 0)) @?~= (  (-1) *^ (cross ((xb fI bI) - (xsb fI bI)) (((m bI) * g) *^ (V3 0 (-1) 0)))  ))
                               
                               ]
                            
                            ,"Gravity = 10 m/s2, freefall 1 link horizontal chain" ~: let
                              mdv = g10Drol1LinkChainMDV
                              MotionDataVars{_bN=bN,_js=js,_bs=bs,_l'=l',_m=m,_xb=xb,_xsb=xsb,_q'=q',_q''=q''} = mdv
                              fI = F 4
                              bI = B 0
                              jI = J 0
                              g = 10
                              ((jointTorques,_),(axb,(xfOj,(ixpi,(inert,(ixpiF,(v,(a,(f',(fNet,_)))))))))) = inverseDynamicsInternals False g mdv fI
                               in test [
                               
                                ("DATA CHECK! accelerations are " ++ show g ++ " ?") ~:
                                        ((l' fI bI) @?~= (vec3 0 (-g) 0))
                               
                                ,"DATA CHECK! bN is 1 ?" ~:
                                        bN @?= 1
                               
                                ,"DATA CHECK! xb of (J 0) is (zero,(5,_,_)) ?" ~:
                                        ( (vx (xb (F 0) bI)) @?~= 5)
                               
                                ,"DATA CHECK! q'' of (J 0) is (zero,(0,-10,0)) ?" ~:
                                        ( (q'' fI jI) @?~= (vec3 0 0 0, vec3 0 (-10) 0))
                                        
                                {-
                               ,"The bone has just gravity as fNet = ???" ~:
                                        (((fNet bI) @?~=: (fromList [0,0,0,0,0,0])) ("bI = " ++ show bI))
                                -}
                                        
                               ,"The bones net force - force gravity = f' = 0" ~:
                                        (((f' bI) @?~=: (fromList [0,0,0,0,0,0])) ( "m = " ++ show (m bI) ++
                                                                                    "\na = " ++ show (a bI) ++
                                                                                    "\nv = " ++ show (v bI) ++
                                                                                    "\ninert bI = " ++ show (inert bI) ++
                                                                                    "\n((inert bI) !* (a bI)) = " ++ show ((inert bI) !* (a bI)) ++
                                                                                    "\n((inert bI) !* (v bI)) = " ++ show ((inert bI) !* (v bI)) ++
                                                                                    "\nfNet = " ++ show (fNet bI) ++
                                                                                    "\nbI = " ++ show bI))
                               
                                ,"torque at J 0 (the only joint) is 0" ~: 
                                        ( (jointTorques (J 0)) @?~= (zero))
                               
                               ]
                            
                            ,"Gravity = 10 m/s2, static 2 link horizontal chain" ~: let
                              mdv = static2LinkChainMDV
                              MotionDataVars{_js=js,_fs=fs,_bs=bs,_m=m,_xj=xj,_xb=xb,_xsb=xsb,_xeb=xeb,_l'=l',_q'=q',_q''=q''} = mdv
                              fI = F 4
                              g = 10
                              ((jointTorques,_),(axb,(xfOj,(ixpi,(inert,(ixpiF,(v,(a,(f',(fNet,(fOe,()))))))))))) = inverseDynamicsInternals False g mdv fI
                               in test [
                               
                                "DATA CHECK! accelerations are 0 ?" ~:
                                        sequence_ [ (l' fI bI) @?~= (V3 0 0 0) | bI <- bs]
                               
                                ,"DATA CHECK! pos of J i relative to J i-1 is (V3 1 0 0)" ~:
                                        sequence_ [((xj fI (J i)) ^-^ (xj fI (J (i-1)))) @?~= (V3 1 0 0) | (J i) <- tail js]
                               
                                ,"q' = (V3 0 0 0, V3 0 0 0)" ~:
                                        sequence_ [(q' fI jI) @?~= (V3 0 0 0, V3 0 0 0) | jI <- js, fI <- fs]
                               
                                ,"q'' = (V3 0 0 0, V3 0 0 0)" ~:
                                        sequence_ [(q'' fI jI) @?~= (V3 0 0 0, V3 0 0 0) | jI <- js, fI <- fs]
                               
                                ,"v = (0, 0, 0, 0, 0, 0)" ~:
                                        sequence_ [(v bI) @?~= (fromList [0,0,0,0,0,0]) | bI <- bs, fI <- fs]
                                
                                ,"Changing to a ixpi (B 1) works as expected" ~:
                                    (((ixpi (B 1)) !* (fromList [1,2,3, 0,0,0])) @?~= (fromList [1,2,3, 0,3,-2]))
                                        
                                ,"Check that ixpiF (B 1) works as expected" ~:
                                        ( ((ixpiF (B 1)) !* (fromList [0,0,0, 0,1,0])) @?~= (fromList [0,0,-1, 0,1,0]) )
                                        
                                -- note that the bone density decreses as you move away from the root. hence CoM must be calculated 
                                ,"torque at J 0 is as expected" ~: let m0 = m (B 0); m1 = m (B 1); x0 = xb fI (B 0); x1 = xb fI (B 1) in
                                        (((jointTorques (J 0)) @?~=: ((-1) *^ (((((x0 ^* m0) + (x1 ^* m1)) ^/ (m0 + m1)) ^-^ (xsb fI (B 0))) `cross` (V3 0 (-(g * ((m (B 0)) + (m (B 1))))) 0) ))) (
                                            "((xfO (B 0)) !* (fOe (B 0))) = " ++ show ((xfOj (J 0)) !* (fOe (B 0))) ++
                                            "\n((xfO (B 1)) !* (fOe (B 1))) = " ++ show ((xfOj (J 1)) !* (fOe (B 1)))
                                         ))
                                        
                                ,"torque at J 1 is as expected" ~: 
                                        (( (jointTorques (J 1)) @?~=: ((-1) *^ ((((xeb fI (B 1)) - (xsb fI (B 1))) ^/ 2) `cross` (V3 0 (-(g * (m (B 1)))) 0)) )) (
                                            "((xfO (B 1)) !* (fOe (B 1))) = " ++ show ((xfOj (J 1)) !* (fOe (B 1)))
                                         ))
                               
                               ]
            ]
            
            -- Test Cases for HUnit extensions
            ,"HUnit extensions" ~: test [
                             "~<?" ~:  1 @<? (2 :: Double)
                            ,"~?<" ~:  1 @?< (2 :: Double)
                            ,"@~=?" ~:  0.000000000000000001 @~=? (0.000000000000000005 :: Double)
                            ,"@~=?" ~:  0.000000000000000005 @~=? (0.000000000000000001 :: Double)
                            ,"@!~=?" ~:  0 @!~=? (1 :: Double)
                            ,"@!~=?" ~:  1 @!~=? (0 :: Double)
            ]
            
        ]
        
    -- print in color! wooop!
    let problems = errors + failures
    putStrLn $ (if problems > 0 
                    then 
                        "\x1b[31m\n\t\tOH NO!!! there's a problem!!!\n"
                    else
                        "\x1b[32m\n\t\t------ SUCCESS ------\n\n\tWell done David! You've earned yourself a break :-D\n") ++ "\x1b[0m"
    
    return ()
    






