:- module(roslogging,
    []).

:- use_module(library(lists), [ member/2 ]).

%%
ros_message_format(debug(Topic,Term),Msg) :-
	!,
	ros_message_format(Term,Msg1),
	ros_message_format((:(Topic,Msg1)),Msg) .

ros_message_format(format(Format,Args),Msg) :-
	!,
	format(atom(Msg),Format,Args).

ros_message_format(Msg,Msg) :-
	atom(Msg),
	!.

ros_message_format(Term,Atom) :-
	term_to_atom(Term,Atom).

%%
ros_message_hook(Predicate,Term) :-
    ros_message_format(Term,Msg),
    call(Predicate,Msg).

%%
% define message_hook clauses to pass pl messages to ROS system.
%
user:message_hook(_, Level, [X|Xs]) :-
	forall(
		member(Format-Args,[X|Xs]),
		user:message_hook(format(Format,Args), Level, [])
	),!.
user:message_hook(Term, error, _)         :- ros_message_hook(ros_error,Term).
user:message_hook(Term, warning, _)       :- ros_message_hook(ros_warn,Term).
user:message_hook(Term, informational, _) :- ros_message_hook(ros_info,Term).
user:message_hook(Term, debug(Topic), _)  :- ros_message_hook(ros_debug,debug(Topic,Term)).

