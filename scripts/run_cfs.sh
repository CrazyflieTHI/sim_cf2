#!/bin/bash
# Handler setup helper script
# Usage:
# ./run_cfs.sh <num_crazyflies>

# ------------------------------
# Resolve script location using :
# https://stackoverflow.com/a/246128/7002298
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done

cf_executable=""
# Check if cf_location is empty
if [ -z "$cf_executable" ]; then
	echo " "
    echo "Error: Please provide the path to the cf2.elf file in this script file directly (cf_executable)"
	echo "e.g. cf_executable="/home/crazyuser/repos/crazyflie-firmware/build/cf2.elf""
	echo " "
    exit 1  # Exit with an error status
fi

cf2="$cf_executable"

# -----------------------
# setup the script so a Ctrl-C call will kill spawned cf2 instance
spawned_cf2_pids=()
function sigint_handler
{
	echo "------"
	echo "run_cfs got SIGINT... killing spawned process"
	for pid in ${spawned_cf2_pids[*]}; do
		echo "killing ${pid}"
		kill $pid
	done
	exit 0
}
trap sigint_handler SIGINT

# ----------------------------
# Processing arguments - setting default values if needed
# number of crazyflies
if [ -z "$1" ]
then
	max_cfs=1
else
	max_cfs=$1
fi

# offset for cf indices
if [ -z "$4" ]
then
	first_cf_index=1
else
	first_cf_index=$4
fi

counter=1
while [ $counter -le $max_cfs ]
do
	cf_index=$((counter + first_cf_index - 1))
	echo "----------------------"
	echo "Spawning cf${cf_index}"
	$cf2 $cf_index &
	spawned_cf2_pids+=($!)
	sleep 0.3
	((counter++))
done

# wait for all pids
echo "spawned process IDs: "
for pid in ${spawned_cf2_pids[*]}; do
	echo $pid
done
for pid in ${spawned_cf2_pids[*]}; do
    wait $pid
done
