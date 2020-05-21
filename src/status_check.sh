#!/bin/bash
# BHG status request script

# Show the number of files saved in each subdirectory
# TODO make this check the latest directory in Data
echo ===== Image File Save Check =====
cd /home/user1/Data/latest
echo "Number files found in directories:"
for f in *; do [ -d ./"$f" ] && find ./"$f" -exec echo \;|wc -l|tr '\n' ' ' && echo $f; done|awk '{print $2"\t"$1}'
echo -

# Show the last lat/long and alt in each csv file
# TODO substitute directory for the latest directory in Data
echo ===== CSV file Data Check =====
for f in *.csv; do 
    SHORTFN=$(echo $f | cut -d '_' -f4); 
    MEASUREMNTS=$(tail -2 $f | head -n1) # | cut -d ',' -f9,10,11)
    LAT=$(echo "$MEASUREMNTS" | cut -d ',' -f9)
    LON=$(echo "$MEASUREMNTS" | cut -d ',' -f10)
    ALT=$(echo "$MEASUREMNTS" | cut -d ',' -f11)
    echo $SHORTFN ----- LAT: $LAT ----- LON: $LON ----- ALT: $ALT
done
echo -

# Show disk usage statistics
# TODO make sure this is the correct drive to check
echo ===== Disk Drive Usage Check =====
df -h /dev/nvme0n1p2
echo -

# Check if MAVROS is being published.
echo ===== MAVROS Ping Check =====
rosnode ping -c 3 /mavros
echo -







