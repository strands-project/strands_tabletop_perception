# QSR KB and Visualisation 

Knowledge representation of and reasoning about QSRs using SWI-Prolog. Specialized predicades can be used to visualize QSRs in RVIZ.  

## Starting the KB service

First the knowledge base service has to be started:
```
roslaunch qsr_kb kb_bringup.launch
```
(Please note that SWI-Prolog needs to be installed; on Ubuntu run: `sudo apt-get install swi-prolog`)

The knowledge base service provides a simple *tell and ask* interface to a Prolog engine. Queries and answers are exchanged using strings.

## Adding knowledge about QSRs to the KB

New perception events with different classifications can be added using the *create_event(Evt,QSRs,Loc,Classifications)* predicate, where *Evt* denotes the event that will be generated, *QSRs* denotes a list of QSRs between segmented object clusters, *Loc* denotes a location where the event occured, and *Classifications* denotes a list of different classifiaction results (incl. probabilities) for all segemented object clusters. 
```
create_event(EVT,
             % QSRs
             [['left-of', 'keyboard', 'cup'], ['left-of', 'monitor', 'cup'],
              ['behind', 'keyboard', 'cup'], ['in-front-of', 'monitor', 'cup'],
              ['in-front-of', 'keyboard', 'monitor'], ['right-of', 'cup', 'monitor']],
             % Loc
             'table27',
             % Classifications (here bottom-up [BU], and top-down [TD])
             [['BU', [['keyboard', 'Keyboard', 0.8], ['keyboard', 'Monitor', 0.2], 
                      ['cup', 'Cup', 0.4], ['cup', 'Mouse', 0.5], ['cup', 'Keyboard', 0.1], 
                      ['monitor', 'Keyboard', 0.1], ['monitor', 'Monitor', 0.9], 
                      ['mouse', 'Cup', 0.9], ['mouse', 'Mouse', 0.1]]], 
              ['TD', [['keyboard', 'Keyboard', 0.9], ['keyboard', 'Monitor', 0.1], 
                      ['cup', 'Cup', 0.6], ['cup', 'Mouse', 0.2], ['cup', 'Keyboard', 0.2], 
                      ['monitor', 'Keyboard', 0.1], ['monitor', 'Monitor', 0.9], 
                      ['mouse', 'Cup', 0.1], ['mouse', 'Mouse', 0.9]]]]).
```

## Querying knowledge about QSRs from the KB and visualise the result.


