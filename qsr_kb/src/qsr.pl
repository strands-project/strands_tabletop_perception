:- use_module(library('gensym')).

:- dynamic
   start_time/2,
   vis_qsr/1.

default_classifier('BU').
%default_classifier('TD').

example1(QSR,LOC,CLASSIFIER_CLASSIFICATION_LST) :-
  QSR = [['left-of', keyboard, cup],['left-of', monitor, cup],['behind', keyboard, cup],['in-front-of', monitor, cup], ['in-front-of', keyboard, monitor],['right-of', cup, monitor]],
  LOC = 'table27',
  CLASSIFIER_CLASSIFICATION_LST = [ ['BU', [[keyboard, 'Keyboard', 0.8], [keyboard, 'Monitor', 0.2], 
                                           [cup, 'Cup', 0.4], [cup, 'Mouse', 0.5],[cup, 'Keyboard', 0.1], 
                                           [monitor, 'Keyboard', 0.1], [monitor, 'Monitor', 0.9],
                                           [mouse, 'Cup', 0.9], [mouse, 'Mouse', 0.1]]] ,
                                    ['TD', [[keyboard, 'Keyboard', 0.9], [keyboard, 'Monitor', 0.1], 
                                         [cup, 'Cup', 0.6], [cup, 'Mouse', 0.2],[cup, 'Keyboard', 0.2], 
                                         [monitor, 'Keyboard', 0.1], [monitor, 'Monitor', 0.9],
                                         [mouse, 'Cup', 0.1], [mouse, 'Mouse', 0.9]]
                                     ]].
  


create_event(EVT, QSR, LOC, CLASSIFIER_CLASSIFICATION_LST) :-
  create_event(EVT),
  add_qsr(EVT,QSR), 
  add_loc(EVT,LOC),
  findall(_,(member(X, CLASSIFIER_CLASSIFICATION_LST), add_classification(EVT, X)),_).

create_event(EVT) :-
  gensym('evt', EVT),
  get_time(Time),
  assertz(start_time(EVT,Time)).

add_qsr(EVT,QSR) :-
  findall(_, (member(X,QSR), assertz(qsr(EVT,X))), _).

add_loc(EVT,LOC) :-
  assertz(at_loc(EVT,LOC)).

add_classification(EVT, [CLASSIFIER, CLASSIFICATION]) :-
  findall(_, (member([OBJ, C, P], CLASSIFICATION), assertz(obj_class(EVT, CLASSIFIER, OBJ, C, P))), _).



most_likely_class(OBJ, CLS) :-
  most_recent(EVT),
  default_classifier(CLASSIFIER),
  most_likely_class(EVT, CLASSIFIER, OBJ, CLS).

most_likely_class(CLASSIFIER, OBJ, CLS) :-
  most_recent(EVT),
  most_likely_class(EVT, CLASSIFIER, OBJ, CLS).

most_likely_class(EVT, CLASSIFIER, OBJ, CLS) :-
  classifier(EVT, CLASSIFIER),
  obj(EVT,OBJ),
  findall([P, OBJ, C], (obj_class(EVT,CLASSIFIER,OBJ,C,P)), Cs),
  sort(Cs, IncCs),
  reverse(IncCs, [[_,OBJ,CLS] | _ ]).
  
most_recent(EVT) :- 
  findall([ST, E], (start_time(E,ST)), Es),
  sort(Es, IncEs),
  reverse(IncEs, [[_,EVT] | _ ]).

obj_cls(OBJ,CLS) :-
  most_recent(EVT),
  default_classifier(CLASSIFIER),
  obj_cls(EVT, CLASSIFIER, OBJ, CLS).

obj_cls(CLASSIFIER, OBJ, CLS) :-
  most_recent(EVT),
  obj_cls(EVT, CLASSIFIER, OBJ, CLS).

obj_cls(EVT, CLASSIFIER, OBJ, CLS) :-
  most_likely_class(EVT, CLASSIFIER, OBJ, CLS).

rel_super_cls(R, RELT) :- 
  member(R, ['left-of', 'right-of', 'behind', 'in-front-of']), 
  RELT = 'DirectionalRelation'.  

rel_super_cls(R, RELT) :-
  member(R, ['close-to', 'distant-to']), 
  RELT = 'DistanceRelation'.  

% qsr related predicates
  
qsr(REL, O1, O2, QSR) :- 
  most_recent(EVT),
  qsr(EVT, REL, O1, O2, QSR).

qsr(EVT, REL, O1, O2, [EVT, 'None', [REL, O1, O2]]) :-
  event(EVT),
  qsr(EVT, [REL, O1, O2]).
  %assertz(vis_qsr(EVT, [REL, O1, O2])).

qsrT(REL, C1, C2, QSR) :- 
  default_classifier(CLASSIFIER),
  qsrT(CLASSIFIER, REL, C1, C2, QSR).

qsrT(CLASSIFIER, REL, C1, C2, QSR) :- 
  most_recent(EVT),
  classifier(EVT, CLASSIFIER),
  qsrT(EVT, CLASSIFIER, REL, C1, C2, QSR).

qsrT(EVT, CLASSIFIER, REL, C1, C2, [EVT, CLASSIFIER, [REL, O1, O2]]) :-
  event(EVT),
  classifier(EVT, CLASSIFIER),
  obj_cls(CLASSIFIER,O1,C1),
  obj_cls(CLASSIFIER,O2,C2),
  qsr(EVT, [REL, O1, O2]).
  %assertz(vis_qsr(EVT, [REL, O1, O2])).

% helper predicates for making predicates generative

event(EVT) :-
  findall(E, start_time(E,_), Es),
  list_to_set(Es,EventSet),
  member(EVT, EventSet).

classifier(EVT, CLASSIFIER) :-
  event(EVT),
  findall(CF, obj_class(EVT,CF,_,_,_), CFs),
  list_to_set(CFs,CFSet),
  member(CLASSIFIER,CFSet).

obj(EVT, OBJ):- 
  event(EVT),
  findall(O, obj_class(EVT,_,O,_,_), Os),
  list_to_set(Os,ObjSet),
  member(OBJ, ObjSet).
  
% init vis_qsr list
vis_qsr([]).

vis(QSR_LST) :-
  vis_qsr(CUR_LST), 
  append(CUR_LST,[QSR_LST],NEW_LST), 
  retract(vis_qsr(CUR_LST)), 
  assertz(vis_qsr(NEW_LST)).

new_vis :- 
  retractall(vis_qsr(_)),
  assertz(vis_qsr([])).

next_vis_2(_) :-
  vis_qsr([]), !, fail.

next_vis_2(QSR_LST) :- 
  vis_qsr([QSR_LST | R]),
  retract(vis_qsr(_)), 
  assertz(vis_qsr(R)).
  
% helper predicates for retrieving visualization info
% get objects
%% get_qsr_lst(QSR_LST) :-  
%%   findall([R, [O1,T1,CLASSIFIER], [O2,T2,CLASSIFIER]], (vis_qsr([E, CLASSIFIER, [R,O1,O2]]), translate(E,CLASSIFIER,O1, T1), translate(E,CLASSIFIER,O2,T2)), QSR_LST).

next_vis(QSR_LST) :-  
  next_vis_2(SOL), 
  findall([R, [O1,T1,CLASSIFIER], [O2,T2,CLASSIFIER]], (member([E, CLASSIFIER, [R,O1,O2]],SOL), translate(E,CLASSIFIER,O1, T1), translate(E,CLASSIFIER,O2,T2)), QSR_LST).


translate(_E, CF, O, O) :-
  CF == 'None'.

translate(E, CF, O, C) :-
  obj_cls(E, CF, O, C).