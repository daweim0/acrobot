#!/bin/bash

# Does bookkeeping

NUM=`ls old_controller/beta* | wc -l`
NEXT=`echo "$NUM + 1" | bc`
CON_DIR=old_controller
INPUTS="beta loghypers training_inputs"

# Backup current controller inputs.
for i in $INPUTS
do
    if [ -e $i.txt ]; then
        mv $i.txt $CON_DIR/$i$NEXT.txt
        hg add $CON_DIR/$i$NEXT.txt
    fi
done

# Save the just-generated training data to a couple places.
scp data/acrobot* jyoungqu@slfiles02:/shareddata/jay/balance_bot/training_data/
hg add data/*

# Pull down the latest set of training data
#TODO

# Update repo
hg ci -m "Routine data collection run"