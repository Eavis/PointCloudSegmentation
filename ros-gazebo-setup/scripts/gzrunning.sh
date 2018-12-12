result=0
for i in `ps -eo comm`; do
  if [ $i = gzserver ]
  then
    result=1
  fi
done
echo $result
