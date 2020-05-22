:- module(rosprolog_query,
    [ rosprolog_query/2,
      rosprolog_encode/2
    ]).
/** <module> Call a goal and get answer as JSON dictionary.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('http/json')).

%% rosprolog_query(+Goal, -Answer) is semidet.
%
% Calls *Goal*, which is an atom representation of
% a term, and JSON encodes the solution and binds
% it to the second argument *Answer*.
%
rosprolog_query('', _) :- !.
rosprolog_query(Goal, Answer) :-
  atom(Goal),
  % parse term from Goal atom
  read_term_from_atom(Goal,GoalTerm,[variable_names(Args)]),
  expand_goal(GoalTerm,Expanded),
  % call the goal
  call(Expanded),
  % create a dictionary
  findall(Name-JSON_value,
    ( member(Name=Value,Args),
      rosprolog_encode(Value,JSON_value)
    ),
    Pairs),
  dict_pairs(Dict,_,Pairs),
  % convert to JSON 
  with_output_to(
    atom(Answer), 
    json_write_dict(current_output, Dict)).

%%
rosprolog_encode([],[]) :- !.

rosprolog_encode([X|Xs],[Y|Ys]) :-
  rosprolog_encode(X,Y),
  rosprolog_encode(Xs,Ys),!.

rosprolog_encode(String,Atom) :-
  string(String),
  term_to_atom(String,Atom), !.

rosprolog_encode(X,X) :-
  ( atom(X) ; is_dict(X) ; number(X) ), !.

rosprolog_encode(Term,_{term: [Functor|Args_json]}) :-
  compound(Term), !,
  Term =.. [Functor|Args_pl],
  rosprolog_encode(Args_pl,Args_json).
  
