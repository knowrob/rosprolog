/*
  Copyright (C) 2017 Daniel Beßler
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

:- module(roscpp,
    [
      ros_publish/3, % TopicName, MsgPath, MsgData
      ros_subscribe/4,
      ros_unsubscribe/2,
      ros_service_call/4,
      ros_package_path/2,
      ros_package_command/2,
      ros_param_get_string/2,
      ros_param_get_double/2,
      ros_param_get_bool/2,
      ros_param_get_int/2,
      ros_param_set_string/2,
      ros_param_set_double/2,
      ros_param_set_int/2,
      ros_param_set_bool/2,
      ros_info/1,
      ros_warn/1,
      ros_error/1,
      ros_debug/1,
      ros_callback/2
    ]).
/** <module> ROS CPP interface for Prolog

@author Daniel Beßler
@license BSD
*/

:- use_module(library('http/json')).

:- dynamic rosprolog_subscriber/2.

user:message_hook(format(X,Args), error, _)         :- ros_message_hook(X,Args,ros_error).
user:message_hook(format(X,Args), warning, -)       :- ros_message_hook(X,Args,ros_warn).
user:message_hook(format(X,Args), informational, _) :- ros_message_hook(X,Args,ros_info).
user:message_hook(format(X,Args), debug(_Topic), _) :- ros_message_hook(X,Args,ros_debug).

%%
ros_message_hook(Format,Args,Predicate) :-
  findall(X, (
    member(Arg,Args),
    (( \+ atom(Arg) ;
       rdf_split_url('',_,Arg) ) ->
      X = Arg ;
      rdf_split_url(_,X,Arg)
    )),
    Args_x),
  format(atom(Msg),Format,Args_x),
  call(Predicate,Msg).

:- use_foreign_library('librosprolog_kb.so').
:- ros_init.

%% ros_publish(+TopicName,+MsgPath,+MsgData) is det.
%
% Publish a message on the topic given in the first argument.
% The message data is supplied as dictionary, where
% the values are pairs of type and value for the corresponding slots.
%
% To publish, for example, a string typed message on the topic
% '/my_topic' one can write:
%
%    ros_publish('/my_topic', 'std_msgs/String', _{data: [string,test]}).
%
% Or by using a list of pairs instead of a dictionary:
%
%    ros_publish('/my_topic', 'std_msgs/String', [data-[string,test]]).
%
% Note that the lowercase *string* refers to the ROS basic datatype string.
% This can be replaced with message types in case a slot refers
% to another message.
%
% A special case is implemented for std_msgs, where the value can be
% provided directly in the third argument:
%
%    ros_publish('/my_topic', 'std_msgs/String', test).
%
ros_publish(TopicName, MsgPath, Dict0) :-
  is_dict(Dict0),!,
  dict_pairs(Dict0,_,Pairs),
  dict_pairs(Dict1,_,[
    msg_path-MsgPath,
    topic_name-TopicName|Pairs]),
  ros_publish_dict(TopicName, Dict1).

ros_publish(TopicName, MsgPath, Pairs) :-
  is_list(Pairs),
  dict_pairs(Dict,_,Pairs),!,
  ros_publish(TopicName, MsgPath, Dict).

% convenience clause for std_msgs allowing
% to leave out the "std_msgs/" prefix
ros_publish(TopicName, StdMsg, Data) :-
  atom_concat('std_msgs/',Type,StdMsg),
  \+ atom_concat(_,'Array',StdMsg), !,
  downcase_atom(Type,TypeLowerCase),
  ros_publish_dict(TopicName, _{
      msg_path: StdMsg,
      topic_name: TopicName,
      data: [TypeLowerCase,Data]
  }).

%%
ros_publish_dict(Topic,Msg_Dict) :-
  with_output_to(atom(Msg_json), 
    json_write_dict(current_output, Msg_Dict)
  ),
  json_ros_publish(Topic,Msg_json).

%% ros_subscribe(+TopicName,+MsgPath,+Callback,-Subscriber) is det.
%
ros_subscribe(TopicName, MsgPath, Callback, Subscriber) :-
  subscriber_name(Subscriber),
  assertz(rosprolog_subscriber(Subscriber,TopicName)),
  %%
  with_output_to(atom(JSON), 
    json_write_dict(current_output, _{
      msg_path: MsgPath,
      topic_name: TopicName
    })
  ),
  json_ros_subscribe(Subscriber,TopicName,JSON,Callback).

%% subscription callback
ros_callback(Goal,Message) :-
  %%
  atom_to_chars(Message,Message_chars),
  open_chars_stream(Message_chars,Message_Stream),
  json_read_dict(Message_Stream,Dict),
  %%
  Goal=..List1,
  ( List1=[':',Mod,Rest] -> (
    Rest=..List2,
    append(List2,[Dict],List3),
    X=..List3,
    Expanded=..[':',Mod,X]
  ) ; (
    append(List1,[Dict],X),
    Expanded=..X
  )),
  % TODO: do some exception handling here
  catch(
    call(Expanded),
    Exc,
    ( writeln(Exc), fail )
  ).

%% ros_unsubscribe(+Subscriber) is det.
%
ros_unsubscribe(Subscriber,TopicName) :-
  with_output_to(atom(JSON), 
    json_write_dict(current_output, _{
      topic_name: TopicName
    })
  ),
  json_ros_unsubscribe(Subscriber,TopicName,JSON),
  %%
  retractall(rosprolog_subscriber(Subscriber,TopicName)).

%%
subscriber_name(N) :-
  random_id_(M),
  ( rosprolog_subscriber(M,_) ->
    subscriber_name(N);
    N=M
  ).

%%
random_id_(Id) :-
  randseq(8, 25, Seq_random),
  maplist(plus(65), Seq_random, Alpha_random),
  atom_codes(Id, Alpha_random).

%% ros_service_call
%
ros_service_call(ServiceName, ServicePath, Request, Response) :-
  with_output_to(atom(Request_json), 
    json_write_dict(current_output, Request)
  ),
  json_ros_service_call(ServicePath,_{
        service_name: ServiceName,
        service_path: ServicePath,
        json_data: Request_json
  }, Response).

%% ros_param_get_string(+Key,-Value) is semidet.
%
% Read parameter from ROS parameter server.

%% ros_param_get_double(+Key,-Value) is semidet.
%
% Read parameter from ROS parameter server.

%% ros_param_get_int(+Key,-Value) is semidet.
%
% Read parameter from ROS parameter server.

%% ros_param_get_bool(+Key,-Value) is semidet.
%
% Read parameter from ROS parameter server.

%% ros_param_set_string(+Key,+Value) is semidet.
%
% Write parameter to ROS parameter server.

%% ros_param_set_double(+Key,+Value) is semidet.
%
% Write parameter to ROS parameter server.

%% ros_param_set_int(+Key,+Value) is semidet.
%
% Write parameter to ROS parameter server.

%% ros_param_set_bool(+Key,+Value) is semidet.
%
% Write parameter to ROS parameter server.

%% ros_info(+Msg) is det.
%
% Debug via ROS master.

%% ros_warn(+Msg) is det.
%
% Debug via ROS master.

%% ros_error(+Msg) is det.
%
% Debug via ROS master.

%% ros_package_path(+Package, -Path) is semidet.
%
% Locate ROS packages on the harddisk using 'rospack find'.

%% ros_package_command(+Command, -Output) is semidet.
%
% Runs a rospack command of the form 'rospack ...'.

%% tf_lookup_transform(+TargetFrame, +SourceFrame, -Transform) is semidet.
%
% True if Transform is the current transform from TargetFrame to SourceFrame.
% Transform is a term "pose([X,Y,Z],[QX,QY,QZ,QW])".

%% tf_listener_start is det.
% 
% Start listening to TF topic.

%% tf_transform_point(+SourceFrame, +TargetFrame, +PointIn, -PointOut) is semidet.
%% tf_transform_quaternion(+SourceFrame, +TargetFrame, +QuaternionIn, -QuaternionOut) is semidet.
%% tf_transform_pose(+SourceFrame, +TargetFrame, +PoseIn, -PoseOut) is semidet.
% 
% Transforms input from SourceFrame to TargetFrame.
% PointIn is a list [float x, y, z].
% QuaternionIn is a list [float qx, qy, qz, qw].
% PoseIn is a term pose([float x, y, z], [float qx, qy, qz, qw]).
