:- module(roslogging,
    [ ros_logger_level/1
    ]).

:- use_module(library(lists), [ member/2, append/2 ]).
% Workaround: See https://github.com/SWI-Prolog/swipl-devel/issues/621
:- append([], _).

%%
% Set the console logging level for ROS logging commands.
% This is e.g. useful to supress console messages when
% running unit tests.
%
ros_logger_level(debug) :- !, ros_set_logger_level(0).
ros_logger_level(info)  :- !, ros_set_logger_level(1).
ros_logger_level(warn)  :- !, ros_set_logger_level(2).
ros_logger_level(error) :- !, ros_set_logger_level(3).
ros_logger_level(fatal) :- !, ros_set_logger_level(4).
ros_logger_level(none)  :- !, ros_set_logger_level(5).

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
	source_location(FilePath,Line),!,
	file_base_name(FilePath,File),
	ros_message_format(Term,Msg),
	atomic_list_concat([Msg,' (',File,':',Line,')'],'',Msg0),
	call(Predicate,Msg0).

ros_message_hook(Predicate,Term) :-
	ros_message_format(Term,Msg),
	call(Predicate,Msg).

%%
ros_message_hook(_, Level, [X|Xs]) :-
	%%
	findall(Format0, (
	    member(Entry,[X|Xs]),
	    once(( Entry=Format0-_ ; Entry=Format0 ))
	), Formats),
	Formats \= [],
	atomic_list_concat(Formats, ' ', Format),
	%%
	findall(Args0, member(_-Args0,[X|Xs]), ArgsList),
	append(ArgsList,Args),
	%%
	user:message_hook(format(Format,Args), Level, []),
	!.
ros_message_hook(Term, error, _)         :- ros_message_hook(ros_error,Term).
ros_message_hook(Term, warning, _)       :- ros_message_hook(ros_warn,Term).
ros_message_hook(Term, informational, _) :- ros_message_hook(ros_info,Term).
ros_message_hook(Term, debug(Topic), _)  :- ros_message_hook(ros_debug,debug(Topic,Term)).

%%
% define message_hook clauses to pass pl messages to ROS system.
%
user:message_hook(Term, Level, Lines) :-
	%% rostest intercepts these messages
	Term \= plunit(_),
	ros_message_hook(Term, Level, Lines).

