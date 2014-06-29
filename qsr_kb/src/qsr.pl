:- use_module(library('gensym')).

:- dynamic
   start_time/2,
   vis_qsr/1.

default_classifier('BU').
%default_classifier('TD').

example1(QSR,LOC,CLASSIFIER_CLASSIFICATION_LST,POSE) :-
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
                                     ]],
   POSE = [ [monitor, [[1.0,0.0,0.0],[1,0,0,0]]], 
            [cup, [[0.5,1.0,0.0],[1,0,0,0]]], 
            [mouse, [[0.5,-0.5,0.0],[1,0,0,0]]],
            [keyboard, [[0.0,0.0,0.0],[1,0,0,0]]] ].
  


create_event(EVT, QSR, LOC, CLASSIFIER_CLASSIFICATION_LST, POSE) :-
  create_event(EVT),
  add_qsr(EVT,QSR), 
  add_loc(EVT,LOC),
  add_pose(EVT,POSE),
  findall(_,(member(X, CLASSIFIER_CLASSIFICATION_LST), add_classification(EVT, X)),_).

create_event(EVT) :-
  gensym('evt', EVT),
  get_time(Time),
  assertz(start_time(EVT,Time)).

add_qsr(EVT,QSR) :-
  findall(_, (member(X,QSR), assertz(qsr(EVT,X))), _).

add_loc(EVT,LOC) :-
  assertz(at_loc(EVT,LOC)).

add_pose(EVT,POSE) :-
  findall(_, (member(X,POSE), assertz(pose(EVT,X))), _).

add_classification(EVT, [CLASSIFIER, CLASSIFICATION]) :-
  findall(_, (member([OBJ, C, P], CLASSIFICATION), assertz(obj_class(EVT, CLASSIFIER, OBJ, C, P))), _).

most_likely_class(OBJ, CLS, PROB) :-
  most_recent(EVT),
  default_classifier(CLASSIFIER),
  most_likely_class(EVT, CLASSIFIER, OBJ, CLS, PROB).

most_likely_class(CLASSIFIER, OBJ, CLS, PROB) :-
  most_recent(EVT),
  most_likely_class(EVT, CLASSIFIER, OBJ, CLS, PROB).

most_likely_class(EVT, CLASSIFIER, OBJ, CLS, PROB) :-
  classifier(EVT, CLASSIFIER),
  obj(EVT,OBJ),
  findall([P, OBJ, C], (obj_class(EVT,CLASSIFIER,OBJ,C,P)), Cs),
  sort(Cs, IncCs),
  reverse(IncCs, [[PROB,OBJ,CLS] | _ ]).
  
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
  most_likely_class(EVT, CLASSIFIER, OBJ, CLS, _).

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

qsr(EVT, REL, O1, O2, [EVT, 'None', [REL, O1, O2],[P1, P2]]) :-
  event(EVT),
  pose(EVT,[O1, P1]),
  pose(EVT,[O2, P2]),
  qsr(EVT, [REL, O1, O2]).
  %assertz(vis_qsr(EVT, [REL, O1, O2])).

findall_objs(Objs) :- 
  findall([Obj, Pose, Cls, Prob], (most_recent(EVT),  pose(EVT,[Obj, Pose]), most_likely_class(_, Obj, Cls, Prob)), Objs).




findall_qsrT(REL,Qs) :-
  nonvar(REL), R = REL, !,
  findall(Q, qsrT(R,_C1,_C2,Q), Qs).

findall_qsrT(REL, C1, C2, Qs) :-
  nonvar(REL), nonvar(C1), nonvar(C2),  
  R = REL, Cls1 = C1, Cls2 = C2, !,
  findall(Q, qsrT(R,Cls1,Cls2,Q), Qs).


qsrT(REL, C1, C2, QSR) :- 
  default_classifier(CLASSIFIER),
  qsrT(CLASSIFIER, REL, C1, C2, QSR).

qsrT(CLASSIFIER, REL, C1, C2, QSR) :- 
  most_recent(EVT),
  classifier(EVT, CLASSIFIER),
  qsrT(EVT, CLASSIFIER, REL, C1, C2, QSR).

qsrT(EVT, CLASSIFIER, REL, C1, C2, [EVT, CLASSIFIER, [REL, O1, O2], [P1, P2]]) :-
  event(EVT),
  classifier(EVT, CLASSIFIER),
  obj_cls(CLASSIFIER,O1,C1),
  obj_cls(CLASSIFIER,O2,C2),
  pose(EVT,[O1, P1]),
  pose(EVT,[O2, P2]),
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
  findall([R, [O1,T1,CLASSIFIER,P1], [O2,T2,CLASSIFIER,P2]], (member([E, CLASSIFIER, [R,O1,O2],[P1,P2]],SOL), translate(E,CLASSIFIER,O1, T1), translate(E,CLASSIFIER,O2,T2)), QSR_LST).


translate(_E, CF, O, O) :-
  CF == 'None'.

translate(E, CF, O, C) :-
  obj_cls(E, CF, O, C).