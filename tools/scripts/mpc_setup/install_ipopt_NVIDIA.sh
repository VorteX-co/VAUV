# Pass the Ipopt source directory as the first argument
if [ -z $1 ]
then
    echo "Specifiy the location of the Ipopt source directory in the first argument."
    exit
fi
cd $1

prefix=/usr/local
srcdir=$PWD

echo "Building Ipopt from ${srcdir}"
echo "Saving headers and libraries to ${prefix}"

# Copy the file config.guess for the automake version installed. To each folder where exist config.guess in ipopt.
cp /usr/share/automake-1.15/config.guess $srcdir/BuildTools/config.guess
cp /usr/share/automake-1.15/config.guess $srcdir/Ipopt/config.guess
cp /usr/share/automake-1.15/config.guess $srcdir/ThirdParty/ASL/config.guess
cp /usr/share/automake-1.15/config.guess $srcdir/ThirdParty/Blas/config.guess
cp /usr/share/automake-1.15/config.guess $srcdir/ThirdParty/HSL/config.guess
cp /usr/share/automake-1.15/config.guess $srcdir/ThirdParty/Lapack/config.guess
cp /usr/share/automake-1.15/config.guess $srcdir/ThirdParty/Metis/config.guess
cp /usr/share/automake-1.15/config.guess $srcdir/ThirdParty/Mumps/config.guess

# BLAS
cd $srcdir/ThirdParty/Blas
./get.Blas
mkdir -p build && cd build
../configure --prefix=$prefix --disable-shared --with-pic
make install

# Lapack
cd $srcdir/ThirdParty/Lapack
./get.Lapack
mkdir -p build && cd build
../configure --prefix=$prefix --disable-shared --with-pic \
    --with-blas="/usr/local/lib/libcoinblas.a -lgfortran"
make install

# ASL
cd $srcdir//ThirdParty/ASL
./get.ASL

# MUMPS
cd $srcdir/ThirdParty/Mumps
./get.Mumps

# build everything
cd $srcdir
./configure --prefix=$prefix --build=arm-linux coin_skip_warn_cxxflags=yes \
    --with-blas="$prefix/lib/libcoinblas.a -lgfortran" \
    --with-lapack=$prefix/lib/libcoinlapack.a
make
make test

make -j8 install
