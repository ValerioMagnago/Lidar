MATLAB="/usr/local/MATLAB/R2015b"
Arch=glnxa64
ENTRYPOINT=mexFunction
MAPFILE=$ENTRYPOINT'.map'
PREFDIR="/home/marco/.matlab/R2015b"
OPTSFILE_NAME="./setEnv.sh"
. $OPTSFILE_NAME
COMPILER=$CC
. $OPTSFILE_NAME
echo "# Make settings for test" > test_mex.mki
echo "CC=$CC" >> test_mex.mki
echo "CFLAGS=$CFLAGS" >> test_mex.mki
echo "CLIBS=$CLIBS" >> test_mex.mki
echo "COPTIMFLAGS=$COPTIMFLAGS" >> test_mex.mki
echo "CDEBUGFLAGS=$CDEBUGFLAGS" >> test_mex.mki
echo "CXX=$CXX" >> test_mex.mki
echo "CXXFLAGS=$CXXFLAGS" >> test_mex.mki
echo "CXXLIBS=$CXXLIBS" >> test_mex.mki
echo "CXXOPTIMFLAGS=$CXXOPTIMFLAGS" >> test_mex.mki
echo "CXXDEBUGFLAGS=$CXXDEBUGFLAGS" >> test_mex.mki
echo "LD=$LD" >> test_mex.mki
echo "LDFLAGS=$LDFLAGS" >> test_mex.mki
echo "LDOPTIMFLAGS=$LDOPTIMFLAGS" >> test_mex.mki
echo "LDDEBUGFLAGS=$LDDEBUGFLAGS" >> test_mex.mki
echo "Arch=$Arch" >> test_mex.mki
echo OMPFLAGS= >> test_mex.mki
echo OMPLINKFLAGS= >> test_mex.mki
echo "EMC_COMPILER=gcc" >> test_mex.mki
echo "EMC_CONFIG=optim" >> test_mex.mki
"/usr/local/MATLAB/R2015b/bin/glnxa64/gmake" -B -f test_mex.mk
