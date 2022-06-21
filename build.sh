# Input device & version
read -p 'Enter device codename: ' DEVICE
read -p 'Enter kernel upstreamed version: ' VERSION

if [[ $DEVICE != wizard ]]; then
    echo Invalid Device codename. Edith build.sh according to your device codename.
    echo STOP!
    exit 1
fi
echo ''
echo Starting kernel build.
echo ''

# set tools directory path
TOP=$(realpath ../)

# export aosp toolchains path 
export PATH="$TOP/tools/gcc/linux-x86/aarch64/aarch64-linux-android-4.9/bin:$PATH"
export PATH="$TOP/tools/gcc/linux-x86/arm/arm-linux-androideabi-4.9/bin:$PATH"
export PATH="$TOP/tools/clang/host/linux-x86/clang-r416183b/bin:$PATH"
export LD_LIBRARY_PATH="$TOP/tools/clang/host/linux-x86/clang-r416183b/lib64:$LD_LIBRARY_PATH"

# export required kernel flags
export ARCH=arm64
export SUBARCH=ARM64
export CLANG_TRIPLE=aarch64-linux-gnu-
export CROSS_COMPILE=aarch64-linux-android-
export CROSS_COMPILE_ARM32=arm-linux-androideabi-

# clean kernel source and out directory
make clean && make mrproper
rm -rf out

# Clean old builds
rm -rf "$TOP/ZIPS/stock/Image.gz-dtb"
rm -rf "$TOP/ZIPS/Kernel_ZIP/STOCK"

# start make builds
make	O=out \
	clean \
	mrproper \
	CC=clang \
	AR=llvm-ar \
	NM=llvm-nm \
	LD=ld.lld \
	STRIP=llvm-strip \
	OBJDUMP=llvm-objdump \
	OBJCOPY=llvm-objcopy \
	${DEVICE}_defconfig

time	make \
	O=out \
	CC=clang \
	AR=llvm-ar \
	NM=llvm-nm \
	LD=ld.lld \
	STRIP=llvm-strip \
	OBJDUMP=llvm-objdump \
	OBJCOPY=llvm-objcopy \
	--jobs=5

# Zipping kernel build
FILE='out/arch/arm64/boot/Image.gz-dtb'
DIR="$TOP/ZIPS"
if [[ -f "$FILE" ]] 
   then    
   mkdir -p "$TOP/Output/$DEVICE-kernel"
   if [[ ! -d "$DIR" ]] 
      then
      echo ZIPS Folder not found !
      echo Image.gz-dtb file is saved in out/arch/arm64/boot folder.
      exit 1
   else
   mkdir -p "$TOP/ZIPS/Kernel_ZIP/STOCK"
   cp out/arch/arm64/boot/Image.gz-dtb "$TOP/ZIPS/stock"
   cd "$TOP/ZIPS/stock"
   zip -r "$TOP/ZIPS/Kernel_ZIP/STOCK/ElectroWizard_STOCK-v$VERSION.zip" *
   echo ''
   echo 'Kernel Build Successful !!!!'
   fi
else
echo ''
echo 'Kernel build Unsuccessfull !!!!'
echo ''
fi
