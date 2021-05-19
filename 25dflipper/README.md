# 2.5D flipper planning module

#### note:
on point sampling its possible states, we should use both left and right S2 as the pivot



#### Usage:
    0. change the ARENA_NAME in `config.py` as you like

    1. build the dilated map

        `python expand.py`

    2. build the path
        
        `python path_search.py`

        or use 'source get_data.sh' to get all of the exp data

    3. visualize the path (optional)

        `cd vis && python local.py`

    4. path following on real robot

        `python following.py`

        



#### maps:
    3 kind of map will be utlized to make experiment:
        1. step
        2. ramp
        3 cross ramp

