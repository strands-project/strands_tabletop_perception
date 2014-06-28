:- use_module(library(http/thread_httpd)).
:- use_module(library(http/http_dispatch)).
:- use_module(library(http/html_write)).
:- use_module(library(http/json)).
:- use_module(library(http/json_convert)).
:- use_module(library(http/http_json)).
:- use_module(library(http/http_log)).

:- http_handler(root(query), handle_query, []).
:- http_handler(root(add), handle_add, []).
:- http_handler(root(remove), handle_remove, []).
:- http_handler(root(shutdown), handle_shutdown, []).

http_json:json_type('application/json').

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This command to start:

start_server(Port, Password, LogFile) :-
	server(Port, Password, LogFile),
	wait.
	
wait :- thread_get_message(_),
	halt.

server(Port, Password, LogFile) :-
	assert(password(Password)),
	http_server(http_dispatch, [port(Port)]),
	set_setting(http:logfile, LogFile).
	
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
unpack(InProlog, OutTerm, OutBindings) :-
	InProlog = json( [action=Action, password=Password] ),
	password(Password),
	atom_to_term(Action, OutTerm, OutBindings).
	
read_json_request(InRequest, OutProlog) :-
	http_read_json(InRequest, JSON),
	json_to_prolog(JSON, OutProlog).
	
handle_add(Request) :-
	read_json_request(Request, Unpackable),
	unpack(Unpackable, Term, _),
	assert(Term),
	format(atom(StringResult), "~q", Term),
	reply_json( StringResult ).
	
handle_remove(Request) :-
	read_json_request(Request, Unpackable),
	unpack(Unpackable, Term, _),
	( 
		(Term = _/_, abolish(Term)) ;
		retract(Term) 
	),
	format(atom(StringResult), "~q", Term),
	reply_json( StringResult ).
	
jsonize(X, Y) :-
	Y = json( X ).

evaluate_query(InTerm, InBindings, OutStringResult) :-
	Goal =.. [findall, InBindings, InTerm, IR],
	call(Goal),
	sort(IR, Result),
	maplist(jsonize, Result, OutStringResult).
	
handle_query(Request) :-
	read_json_request(Request, Unpackable),
	unpack(Unpackable, Term, Bindings),
	evaluate_query(Term, Bindings, Result),
	reply_json(Result).
	
handle_shutdown(Request) :-
	read_json_request(Request, Unpackable),
	unpack(Unpackable, Term, _),
	format(atom(StringResult), "~q", Term),
	reply_json( StringResult ),
	halt.
	
