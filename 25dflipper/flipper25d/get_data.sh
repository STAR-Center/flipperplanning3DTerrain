for i in {0..9}
do
    FILENAME="ARENA_NAME = STEP_ARENA_NAME_LIST[${i}]"
    echo $FILENAME
    echo $FILENAME >> config.py
    python path_search.py   
 
    FILENAME="ARENA_NAME = RAMP_ARENA_NAME_LIST[${i}]"
    echo $FILENAME
    echo $FILENAME >> config.py
    python path_search.py

    FILENAME="ARENA_NAME = IRAMP_ARENA_NAME_LIST[${i}]"
    echo $FILENAME
    echo $FILENAME >> config.py
    python path_search.py 

done
