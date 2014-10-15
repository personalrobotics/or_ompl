#!/bin/bash

planners=( 'BKPIECE1' 'EST' 'KPIECE1' 'LBKPIECE1' 'PDST' 'PRM' 'PRMstar' 'RRT' 'RRTConnect' 'RRTstar' 'SBL' 'TRRT' )
window_id='0x4800011' # Run 'xwininfo', click on RViz, and record the "Window id".
output_dir='videos'

mkdir -p "${output_dir}"

for planner in ${planners[@]}; do
    echo
    echo "# Running ${planner}"
    echo '#'
    ./scripts/example.py --window-id="${window_id}" --output-dir="${output_dir}" "${planner}"
done
