
#!/bin/bash

#
# error del *.c in follow dir	
# 1. scripts
# 2. kernel
# 3. lib
# 4. trunk/usr
# 5. arch/arm/kernel
# 6. mm
# 7. usr

#p=`cat /home/mgq/Desktop/name.txt`
p=`find -name "*.o" | sed -e 's/\.o/\.c/g'`

for c in $p
do cp -f $c $c".bak.2"
done

find . -name "*.c" | xargs rm -f

p=`find -name "*.bak.2"` 
# | sed -e 's/\.o/\.c/g'`

for c in $p
do tc=$c
 dn=`echo $c |sed -e 's/\.c\.bak\.2/\.c/g'`
#echo $dn
 mv $tc $dn 
done



