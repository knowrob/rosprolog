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
      not(sub_string(Name,0,1,_,'_')),
      rosprolog_encode(Value,JSON_value)
    ),
    Pairs),
  dict_pairs(Dict,_,Pairs),
  % convert to JSON 
  with_output_to(
    atom(Answer), 
    json_write_dict(current_output, Dict)).

%%
rosprolog_encode(EmptyList,[]) :- 
  ground(EmptyList),
  EmptyList=[],!.

rosprolog_encode(List,[Y|Ys]) :-
  % allow lists that contain non-ground entries
  list_parts(List, X, Xs),
  % ground(X),ground(Xs),
  rosprolog_encode(X,Y),
  rosprolog_encode(Xs,Ys),!.

rosprolog_encode(String,Atom) :-
  string(String),
  term_to_atom(String,Atom), !.

rosprolog_encode(X,X) :-
  ( atom(X) ; is_dict(X) ; number(X) ), !.

rosprolog_encode(Term,_{term: [Functor|Args_json]}) :-
  compound(Term),
  Term =.. [Functor|Args_pl],
  % If the Term is a nonempty list, it is a term of the form
  % '[|]'(Head, Tail)
  % Normally lists should be handled above, but when that doesn't happen (for example because there are variables in the list and the old ground-based check is used), this would result in a recursion without a base case.
  Term \== '[|]',
  !,
  rosprolog_encode(Args_pl,Args_json).

rosprolog_encode(Unbound,'_') :-
  not(ground(Unbound)),!.

%% list_parts(@List, -Head, -Tail) is semidet.
%
% Checks if List is a non-empty list and returs the head and tail of the list.
% If List is not sufficiently instantiated, this predicate fails silently. 
list_parts(List, Head, Tail) :-
  compound(List),
  List =.. [Functor, Head, Tail],
  Functor == '[|]'.
