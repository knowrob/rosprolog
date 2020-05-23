:- module(rostest,
    [ rospl_run_tests/2
    ]).
/** <module> Run plunit tests in the scope of a ROS workspace.

@author Daniel Beßler
@license BSD
*/

:- use_module(library(plunit)).
:- expects_dialect(sicstus). % for use_module/3

:- dynamic test_suite_begin/2,
           test_suite_end/2,
           test_case_begin/3,
           test_case_end/3,
           test_case_failure/3,
           out_stream_/1.

%% pretty print some messages
prolog:message(test_failed(Unit, Name, Error)) -->
	[ '[plunit] ~p:~p failed. msg:"~p"'-[Unit,Name,Error] ].
prolog:message(test_nondet(Name)) -->
	[ '[plunit] ~p succeeded with choicepoint'-[Name] ].
prolog:message(test_report(name=N,tests=C0,failures=C1,errors=C2,time=Time)) -->
	[ '[plunit] ~p pass:~p/~p time:~p'-[N,Count,C0,Time] ],
	{ Count is C0 - (C1 + C2) }.

%% Intercept plunit messages, and
%% create facts using the danymic "test_*" predicates.
user:message_hook(plunit(Args), _Level, _Lines) :-
	once( plunit_message_hook(Args) ).
plunit_message_hook(begin(Unit)) :-
	get_time(Time), assertz(test_suite_begin(Unit,Time)).
plunit_message_hook(end(Unit)) :-
	get_time(Time), assertz(test_suite_end(Unit,Time)).
plunit_message_hook(begin(Unit:Test, _File:_Line, _STO)) :-
	unpack_test_(Test,TestA),
	get_time(Time), assertz(test_case_begin(Unit,TestA,Time)).
plunit_message_hook(end(Unit:Test, _File:_Line, _STO)) :-
	unpack_test_(Test,TestA),
	get_time(Time), assertz(test_case_end(Unit,TestA,Time)).
plunit_message_hook(failed(Unit, Name, _Line, Error)) :-
	% need to select the output stream for print_message explicitely in the
	% the scope of *run_tests*.
	out_stream_(OS),
	with_error_to_(OS,
		print_message(error,test_failed(Unit, Name, Error))),
	assertz(test_case_failure(Unit,Name,Error)).
plunit_message_hook(nondet(_,_,Name)) :-
	print_message(warning,test_nondet(Name)).

%% NOTE: @(Test,Args) is used when *forall* is used in test options.
unpack_test_(@(Test,_),Test) :- !.
unpack_test_(Test,Test) :- !.

%%
is_plt_file(File) :-
	file_name_extension(_, plt, File).

%% rospl_run_tests(+Target, +Opts) is det.
%
% Runs tests corresponding to Target.
% Target must be a term "Pkg:Module" where
% Pkg is the name of a ROS package, and Module
% is the path to a Prolog module relative to
% the *prolog* directory of that package.
% Opts is a list of options that may hold the following
% keys:
%
% - xunit(File): write XUnit output to given file.
%
% - report: write a report once all tests finished.
%
rospl_run_tests(Target, _Opts) :-
	\+ atom(Target), !,
	throw(invalid_argument(rospl_run_tests,Target)).

rospl_run_tests(Target, Opts) :-
	exists_file(Target),!,
	run_tests_(Target,Opts).

rospl_run_tests(Target,Opts) :-
	exists_directory(Target),!,
	run_tests_(Target,Opts).

rospl_run_tests(Target, Opts) :-
	ros_package_path(Target,PkgPath),!,
	atom_concat(PkgPath, '/src/', PlPath),
	exists_directory(PlPath),
	run_tests_(PlPath,Opts).

%%
run_tests_(Directory,Opts) :-
	exists_directory(Directory),!,
	directory_files(Directory,Entries),
	forall(
		( member(Entry,Entries), \+ atom_prefix(Entry,'.') ),
		( atomic_list_concat([Directory,Entry],'/',Child),
			( exists_directory(Child) -> run_tests_(Child,Opts)
			; is_plt_file(Child)      -> run_test_(Child,Opts)
			; true )
		)
	).

run_tests_(Target,Opts) :-
	run_test_(Target,Opts)
	-> true
	;  throw(invalid_argument(run_tests_,Target)).

%%
run_test_(TestFile, Opts) :-
	file_name_extension(X, plt, TestFile),!,
	file_name_extension(X, pl,  ModuleFile),
	run_test_(ModuleFile, Opts).

run_test_(ModuleFile, Opts) :-
	file_name_extension(_, pl,  ModuleFile),
	%%
	directory_file_path(FileDir, _, ModuleFile),
	( get_package_path_(FileDir,PackagePath)
	-> init_ros_package(PackagePath)
	;  true
	),
	%%
	catch(
		run_test__(ModuleFile, Opts),
		Error,
		print_message(error,test_failed(ModuleFile, '*', Error))
	).

run_test__(ModuleFile, Opts) :-
	% load files
	use_module(Module,ModuleFile,[]),
	load_test_files(_),
	% remember old user output
	stream_property(OldOut, alias(user_output)),
	retractall(out_stream_(_)),
	assertz(out_stream_(OldOut)),
	%% run test and report
	setup_call_cleanup(
		%% setup
		true,
		%% call
		( ignore(run_tests([Module]))
		, forall(test_report_(Module,Opts),true)
	 	),
	 	%% cleanup
	 	( test_suite_retract_(Module) )
	).

%%
get_package_path_(Directory,PkgPath) :-
	atomic_list_concat(Entries,'/',Directory),
	append(PrefixEntries,[src|Suffix],Entries),
	\+ memberchk(src, Suffix),
	append(PrefixEntries,[src,''],X),
	atomic_list_concat(X,'/',PkgPath),!.

%% retract dynamic facts
test_suite_retract_(Module) :-
	retractall(test_suite_begin(Module,_)),
	retractall(test_suite_end(Module,_)),
	retractall(test_case_begin(Module,_,_)),
	retractall(test_case_end(Module,_,_)),
	retractall(test_case_failure(Module,_,_)).

%% make a call but redirect *user_error* to another stream.
with_error_to_(Stream,Goal) :-
	stream_property(OldErr, alias(user_error)),
	set_stream(Stream, alias(user_error)),
	call(Goal),
	set_stream(OldErr, alias(user_error)).

%%
test_report_(Module,Opts) :-
	member(xunit(File),Opts),
	test_report_xunit_(Module,File).

test_report_(Module,Opts) :-
	member(report,Opts),
	xunit_term_(Module,element(testsuite,Args,_Body)),
	X=..[test_report|Args],
	print_message(informational,X).

%%
test_report_xunit_(Module,File) :-
	xunit_term_(Module,Term),
	open(File,write,Stream), 
	xml_write(Stream, [Term], [layout(true)]),
	close(Stream).

% XUnit term generator
xunit_term_(Module, element(testsuite,
		[ name=Module, tests=NumTests,
		  failures=NumFailures, errors=NumErrors,
		  time=TestTime ], TestCaseTerms)) :-
	%%
	test_suite_begin(Module,T0),
	test_suite_end(Module,T1),
	TestTime is T1 - T0,
	%%
	findall(X0, test_case_begin(Module,X0,_), TestCases),
	length(TestCases,NumTests),
	%%
	findall(X1, (
		test_case_failure(Module,X1,Failure),
		xunit_is_failure_(Failure)
	), Failures),
	length(Failures,NumFailures),
	%%
	findall(X2, (
		test_case_failure(Module,X2,Err),
		xunit_is_error_(Err)
	), Errors),
	length(Errors,NumErrors),
	%%
	findall(TestTerm, (
		member(X3, TestCases),
		xunit_test_term_(Module,X3,TestTerm)
	), TestCaseTerms).

xunit_test_term_(Module,TestCase,
	element(testcase,
		[ name=TestCase, time=TestTime ],
		FailureTerms)) :-
	%%
	test_case_begin(Module,TestCase,T0),
	test_case_end(Module,TestCase,T1),
	TestTime is T1 - T0,
	%%
	findall(FailureTerm, (
		test_case_failure(Module,TestCase,Failure),
		xunit_failure_term_(Failure,FailureTerm)
	), FailureTerms).

%%
xunit_is_failure_(failed).
xunit_is_failure_(succeeded(_)).
xunit_is_error_(X) :- \+ xunit_is_failure_(X).

%%
xunit_failure_term_(failed,
	element(failure, [ type=failed, message='goal failed' ], [])) :- !.

xunit_failure_term_(succeeded(_),
	element(failure, [ type=failed,
	message='goal succeeded but should have failed' ], [])) :- !.

xunit_failure_term_(Error,
	element(error, [ type=exception, message=Msg ], [])) :-
	atom(Error) -> Msg = Error ; term_to_atom(Error,Msg).
