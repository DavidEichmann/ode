module Paths_ODEHask (
    version,
    getBinDir, getLibDir, getDataDir, getLibexecDir,
    getDataFileName
  ) where

import qualified Control.Exception as Exception
import Data.Version (Version(..))
import System.Environment (getEnv)
import Prelude

catchIO :: IO a -> (Exception.IOException -> IO a) -> IO a
catchIO = Exception.catch


version :: Version
version = Version {versionBranch = [0,1], versionTags = []}
bindir, libdir, datadir, libexecdir :: FilePath

bindir     = "/home/david/.cabal/bin"
libdir     = "/home/david/.cabal/lib/ODEHask-0.1/ghc-7.6.3"
datadir    = "/home/david/.cabal/share/ODEHask-0.1"
libexecdir = "/home/david/.cabal/libexec"

getBinDir, getLibDir, getDataDir, getLibexecDir :: IO FilePath
getBinDir = catchIO (getEnv "ODEHask_bindir") (\_ -> return bindir)
getLibDir = catchIO (getEnv "ODEHask_libdir") (\_ -> return libdir)
getDataDir = catchIO (getEnv "ODEHask_datadir") (\_ -> return datadir)
getLibexecDir = catchIO (getEnv "ODEHask_libexecdir") (\_ -> return libexecdir)

getDataFileName :: FilePath -> IO FilePath
getDataFileName name = do
  dir <- getDataDir
  return (dir ++ "/" ++ name)
