#!/usr/bin/env python3
import re
import os
import traceback

from builtins import input

import rospy
import readline
import sys
import json
from rosprolog_client import PrologException, Prolog, PrologQuery
from json_prolog_msgs import srv

from flask import Flask, jsonify, request
from flask_restful import reqparse, abort, Api, Resource

app = Flask(__name__)
# api = Api(app)

parser = reqparse.RequestParser()
parser.add_argument('query', type=str)
parser.add_argument('id', type=int)

class RosprologRestClient:
    def __init__(self, name_space='rosprolog', timeout=None, wait_for_services=True):
        """
        :type name_space: str
        :param timeout: Amount of time in seconds spend waiting for rosprolog to become available.
        :type timeout: int
        """
        self._simple_query_srv = rospy.ServiceProxy('{}/query'.format(name_space), srv.PrologQuery)
        self._next_solution_srv = rospy.ServiceProxy('{}/next_solution'.format(name_space), srv.PrologNextSolution)
        self._finish_query_srv = rospy.ServiceProxy('{}/finish'.format(name_space), srv.PrologFinish)
        if wait_for_services:
            rospy.loginfo('waiting for {} services'.format(name_space))
            self._finish_query_srv.wait_for_service(timeout=timeout)
            self._simple_query_srv.wait_for_service(timeout=timeout)
            self._next_solution_srv.wait_for_service(timeout=timeout)
            rospy.loginfo('{} services ready'.format(name_space))

    def query(self, in_id, query_str):
        result = self._simple_query_srv(id=in_id, query=query_str, mode=1)
        rospy.loginfo(str(query_str))
        if not result.ok:
            return jsonify(sucess=False)
        return jsonify(sucess=True)

    def next(self, in_id):
        next_solution = self._next_solution_srv(id=in_id)
        rospy.loginfo(str(next_solution))
        if not next_solution.status == srv.PrologNextSolutionResponse.OK:
            return jsonify(sucess=False)
        return json.loads(next_solution.solution)

    def all(self, in_id):
        solutions = list()
        finished = False
        try:
            while not finished:
                next_solution = self._next_solution_srv(id=in_id)
                if next_solution.status == srv.PrologNextSolutionResponse.OK:
                    solutions.append(next_solution.solution)
                elif next_solution.status == srv.PrologNextSolutionResponse.NO_SOLUTION:
                    break
                else:
                    return jsonify(sucess=False)
        finally:
            self.finish(in_id)
        return json.dumps(solutions)

    def finish(self, in_id):
        self._finish_query_srv(id=in_id)
        return jsonify(sucess=True)


# Test call with:
# curl --header "Content-Type: application/json"   --request POST   --data '{"query":"has_type(A,B).","id":"1"}'   http://localhost:62226/knowrob/api/v1.0/query
@app.route('/knowrob/api/v1.0/query', methods=['POST'])
def query_rest():
    request.get_json(force=True)
    args = parser.parse_args()
    query_str = str(args['query'])
    in_id = str(args['id'])
    return rosrest.query(in_id, query_str)


# Test call with:
# curl --header "Content-Type: application/json"   --request GET   --data '{"id":"1"}'   http://localhost:62226/knowrob/api/v1.0/next_solution
@app.route('/knowrob/api/v1.0/next_solution', methods=['GET'])
def next_rest():
    request.get_json(force=True)
    args = parser.parse_args()
    in_id = str(args['id'])
    return rosrest.next(in_id)


# Test call with:
# curl --header "Content-Type: application/json"   --request GET   --data '{"id":"1"}'   http://localhost:62226/knowrob/api/v1.0/all_solutions
@app.route('/knowrob/api/v1.0/all_solutions', methods=['GET'])
def all_rest():
    request.get_json(force=True)
    args = parser.parse_args()
    in_id = str(args['id'])
    return rosrest.all(in_id)


# Test call with:
# curl --header "Content-Type: application/json"   --request POST   --data '{"id":"1"}'   http://localhost:62226/knowrob/api/v1.0/finish
@app.route('/knowrob/api/v1.0/finish', methods=['POST'])
def finish_rest():
    request.get_json(force=True)
    args = parser.parse_args()
    in_id = str(args['id'])
    return rosrest.finish(in_id)


if __name__ == '__main__':
    rospy.init_node('rosprolog_rest', anonymous=True)
    rosrest = RosprologRestClient()
    app.run(host='0.0.0.0',debug=True,port=62226)
