import Data.List


type Dat = [Double]
type Dats = [[Double]]






saveDir = "/home/david/Documents/File Dump/git/ode/ODE_01/Data/Animation/simple/punches/"

-- take an input file and apply the 2 legs to it and save in
leggify x y z file outputPrefix = do
	input <- readFile file
	let
		dats = getDats input
		leggified1 = applyLegs1To x y z dats
		leggified2 = applyLegs2To x y z dats
	write leggified1 (saveDir ++ outputPrefix ++ "1.bvh")
	write leggified2 (saveDir ++ outputPrefix ++ "2.bvh")


write :: Dats -> String -> IO ()
write dat file = writeFile file $ heirarchyString ++ (intercalate "\n" (map (intercalate " ") (map (map show) dat))) ++ "\n"

applyLegs1To x y z = applyLegsTo x y z staticLegs1
applyLegs2To x y z = applyLegsTo x y z staticLegs2

applyLegsTo :: Double -> Double -> Double -> Dat -> Dats -> Dats
applyLegsTo xRot yRot zRot legDat upperDat  = map (\l ->
  -- root rotation
  ([xRot,0,zRot] ++ (replicate 3 0)) ++ 
  -- upper body y rotation added to spine
  [l!!6 + l!!3 + yRot,  l!!7,  l!!8] ++
  -- upper body
  (drop 9 $ take (72-24) l) ++ 
  -- legs
  (drop (72-24) legDat)) upperDat


getDats :: String -> Dats
getDats input = filter ((== 72) . length) $ map ((map read) . words) $ lines input

staticLegs1 :: Dat
staticLegs1 = head $ getDats "-53.260953 86.684657 72.101693 -2.983738 -7.258926 -0.709959 -0.678256 3.167840 -0.447900 -0.293252 1.419085 -0.245960 -0.293252 1.419085 -0.245960 -0.219356 1.064433 -0.183780 0.469101 17.623769 0.058340 0.845611 -3.455132 -0.051888 -2.501880 -0.626007 9.379212 -6.306777 -2.656480 72.780161 18.072550 18.110145 10.772590 -3.261163 2.818312 -8.686108 -3.191877 -0.178063 -5.356430 -1.159806 -8.549995 -71.452115 -15.029136 25.438627 -17.572724 3.948968 5.412671 15.612117 0 10 -20 0 0 0 0 0 20 0 0 0 0 10 20 0 0 0 0 0 -20 0 0 0"

staticLegs2 :: Dat
staticLegs2 = head $ getDats "0 91.00 0 0 0 0 -0.648814 3.154718 -0.462364 -0.281707 1.413576 -0.248323 -0.281707 1.413576 -0.248323 -0.210695 1.060297 -0.185581 0.463178 17.639046 0.074295 0.829203 -3.426891 -0.020349 -2.541352 -0.627069 9.398110 -6.213603 -2.706069 72.927478 18.277639 18.066041 10.857052 -3.289142 2.783730 -8.591631 -3.231513 -0.170092 -5.356287 -1.311545 -8.579991 -71.564476 -15.264066 25.455514 -17.773698 3.930610 5.407616 15.570784 0 15 -5 0 0 0 0 0 0 -90 0 0 0 -15 5 0 0 0 0 0 0 -90 0 0"

heirarchyString = "HIERARCHY\nROOT Hips\n{\n OFFSET 0.000000 0.000000 0.000000\n CHANNELS 6 Xposition Yposition Zposition Yrotation Xrotation Zrotation\n JOINT Chest\n {\n  OFFSET 0.000000 10.080317 -0.001081\n  CHANNELS 3 Yrotation Xrotation Zrotation\n  JOINT Chest2\n  {\n   OFFSET 0.000000 11.096852 0.001963\n   CHANNELS 3 Yrotation Xrotation Zrotation\n   JOINT Chest3\n   {\n    OFFSET 0.000000 10.026525 0.000000\n    CHANNELS 3 Yrotation Xrotation Zrotation\n    JOINT Chest4\n    {\n     OFFSET 0.000000 10.026525 0.000000\n     CHANNELS 3 Yrotation Xrotation Zrotation\n     JOINT Neck\n     {\n      OFFSET 0.000000 14.108171 0.000000\n      CHANNELS 3 Yrotation Xrotation Zrotation\n      JOINT Head\n      {\n       OFFSET 0.000000 9.333493 0.004668\n       CHANNELS 3 Yrotation Xrotation Zrotation\n       End Site\n       {\n        OFFSET 0.000000 17.851786 0.002438\n       }\n      }\n     }\n     JOINT RightCollar\n     {\n      OFFSET -3.071622 7.873185 0.000000\n      CHANNELS 3 Yrotation Xrotation Zrotation\n      JOINT RightShoulder\n      {\n       OFFSET -14.378659 0.000000 0.000000\n       CHANNELS 3 Yrotation Xrotation Zrotation\n       JOINT RightElbow\n       {\n        OFFSET -30.778728 0.000000 0.000000\n        CHANNELS 3 Yrotation Xrotation Zrotation\n        JOINT RightWrist\n        {\n         OFFSET -25.164217 0.000000 0.000000\n         CHANNELS 3 Yrotation Xrotation Zrotation\n         End Site\n         {\n          OFFSET -18.796788 0.000000 0.000000\n         }\n        }\n       }\n      }\n     }\n     JOINT LeftCollar\n     {\n      OFFSET 3.071622 7.873185 0.000000\n      CHANNELS 3 Yrotation Xrotation Zrotation\n      JOINT LeftShoulder\n      {\n       OFFSET 14.378659 0.000000 0.000000\n       CHANNELS 3 Yrotation Xrotation Zrotation\n       JOINT LeftElbow\n       {\n        OFFSET 30.778728 0.000000 0.000000\n        CHANNELS 3 Yrotation Xrotation Zrotation\n        JOINT LeftWrist\n        {\n         OFFSET 25.164217 0.000000 0.000000\n         CHANNELS 3 Yrotation Xrotation Zrotation\n         End Site\n         {\n          OFFSET 18.796788 0.000000 0.000000\n         }\n        }\n       }\n      }\n     }\n    }\n   }\n  }\n }\n JOINT RightHip\n {\n  OFFSET -8.194923 0.005295 0.000540\n  CHANNELS 3 Yrotation Xrotation Zrotation\n  JOINT RightKnee\n  {\n   OFFSET 0.000000 -42.473320 -0.000511\n   CHANNELS 3 Yrotation Xrotation Zrotation\n   JOINT RightAnkle\n   {\n    OFFSET 0.000000 -41.474259 -0.000865\n    CHANNELS 3 Yrotation Xrotation Zrotation\n    JOINT RightToe\n    {\n     OFFSET 0.000000 -7.354994 16.712725\n     CHANNELS 3 Yrotation Xrotation Zrotation\n     End Site\n     {\n      OFFSET 0.000000 -1.553903 6.764158\n     }\n    }\n   }\n  }\n }\n JOINT LeftHip\n {\n  OFFSET 8.194923 0.005295 0.000540\n  CHANNELS 3 Yrotation Xrotation Zrotation\n  JOINT LeftKnee\n  {\n   OFFSET 0.000000 -42.473320 -0.000511\n   CHANNELS 3 Yrotation Xrotation Zrotation\n   JOINT LeftAnkle\n   {\n    OFFSET 0.000000 -41.474259 -0.000865\n    CHANNELS 3 Yrotation Xrotation Zrotation\n    JOINT LeftToe\n    {\n     OFFSET 0.000000 -7.354994 16.712725\n     CHANNELS 3 Yrotation Xrotation Zrotation\n     End Site\n     {\n      OFFSET 0.000000 -1.553903 6.764158\n     }\n    }\n   }\n  }\n }\n}\nMOTION\nFrames: 345\nFrame Time: 0.016667\n"

