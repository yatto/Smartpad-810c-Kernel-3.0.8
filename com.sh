
#!/bin/bash

proname=$1
assname="_"`svn info|grep "Author:"|awk -F ": " '{print$2}'`
assname=$assname"_r"`svn info|grep "Rev:"|awk -F ": " '{print$2}'`
prosrc="arch/arm/boot/Image"
prodest="/mnt/hgfs/f/Image_"$1$assname

echo $prodest


echo $1

./build_kernel.sh $proname && 
cp -f $prosrc $prodest && 
echo "cp Image to "$prodest

echo "build finish!"

