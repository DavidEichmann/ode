module Motion.Joint where

import Util
import Linear (zero)
import Data.List (intersect)

data Joint = Joint {
            name       :: String,
            offset     :: Vec3,
            rotationL  :: Quat,
            channels   :: [Channel]
    } deriving (Show, Eq)
    
data Channel = Xpos | Ypos | Zpos | Xrot | Yrot | Zrot deriving (Eq, Show)
posChans :: [Channel]
posChans = [Xpos,Ypos,Zpos]
rotChans :: [Channel]
rotChans = [Xrot,Yrot,Zrot]
    

nullJoint :: Joint
nullJoint = Joint {
        name = "",
        offset = zero,
        rotationL = identity,
        channels = []
}

numChannels :: Joint -> Int
numChannels = length . channels

hasPosChan :: Joint -> Bool
hasPosChan j = not $ null (intersect posChans (channels j))

hasRotChan :: Joint -> Bool
hasRotChan j = not $ null (intersect rotChans (channels j))