:- module(roslogging,
    []).

:- use_module(library(lists), [ member/2, append/2 ]).

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

