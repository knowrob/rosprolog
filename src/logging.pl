:- module(roslogging,
    [ ros_logger_level/1
    ]).

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
append_file(Goal,In,Out) :-
	memberchk(Goal,[ros_error,ros_warn]),
	source_location(FilePath,Line),!,
	file_base_name(FilePath,File),
	atomic_list_concat([In,' (',File,':',Line,')'],'',Out).

%%
%
ros_message_hook(Goal,Lines) :-
	print_message_lines(atom(Msg0), '', Lines),
	atom_concat(Msg,'\n',Msg0),
	(	append_file(Goal,Msg,Msg1)
	->	call(Goal,Msg1)
	;	call(Goal,Msg)
	).

%%
%
user:message_hook(log(_), error, Lines)         :- ros_message_hook(ros_error,Lines).
user:message_hook(log(_), warning, Lines)       :- ros_message_hook(ros_warn,Lines).
user:message_hook(log(_), informational, Lines) :- ros_message_hook(ros_info,Lines).
user:message_hook(log(_), debug(_), Lines)      :- ros_message_hook(ros_debug,Lines).
